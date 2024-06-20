#ifndef LINKSOLVER_HPP
#define LINKSOLVER_HPP

#include <ilcplex/ilocplex.h>
#include <vector>
#include <deque>
#include <iostream>

ILOSTLBEGIN

class LinkSolver {
public:
    LinkSolver(const std::deque<std::vector<double>>& Ei_k, 
               const std::deque<std::vector<double>>& Ci_k,
               const std::deque<double>& Ri,
               const std::deque<double>& Di)
        : n(Ei_k.size()), m(Ei_k[0].size()), Ei_k(Ei_k), Ci_k(Ci_k), Ri(Ri), Di(Di), env(), model(env), cplex(model) {
       
            initializeVariables();
            addInitialConstraints();
            addObjective();
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
    }

    struct  SolverResult
	{
		bool solved;
        double objective_value;
		std::vector<std::vector<float>> job_speeds;
	}; 

    SolverResult solve() {
        SolverResult  result; // Init the result

        try {
            if (cplex.solve()) { // Solve the solver
                result.solved = true; // if solution found, then add to result
                result.objective_value = cplex.getObjValue(); // Add objective value to the result
                for (int i = 0; i < n; i++) {
                    // REFACTOR: Energy setting is hardcoded. Need better abstraction
                    std::vector<float> original_speed = {0.6,0.7,0.8,0.9,1.0};
                    for (int k = 0; k < m; k++) {
                      if(cplex.getValue(Si_k[i][k])) { // For each job, cjeck the index of selected speed
                        if (k > 0) original_speed.erase(original_speed.begin(), original_speed.begin() + k);
                        result.job_speeds.push_back(original_speed); // Add speed and higher in the result 
                      }
                    }
                }
            } else {
                result.solved = false;
                std::cout << "No solution found" << std::endl;
            }
        } catch (IloException& e) {
            std::cerr << "Error during optimization: " << e << std::endl;
            
        } catch (...) {
            std::cerr << "Unknown error during optimization" << std::endl;
            
        }
        return result;
    }

    void cleanup() {
        env.end();
    }

private:
    int n, m;
    std::deque<std::vector<double>> Ei_k;
    std::deque<std::vector<double>> Ci_k;
    std::deque<double> Ri;
    std::deque<double> Di;

    IloEnv env;
    IloModel model;
    IloCplex cplex;
    IloArray<IloBoolVarArray> Si_k;
    IloIntVarArray Fi;

    

    void initializeVariables() {
        Si_k = IloArray<IloBoolVarArray>(env, n);
        for (int i = 0; i < n; i++) {
            Si_k[i] = IloBoolVarArray(env, m);
        }
        Fi = IloIntVarArray(env, n, 0, IloInfinity);
    }

    void addInitialConstraints() {
        try {
            // Constraint 1: Sum of Si,k for each job i equals 1
            for (int i = 0; i < n; i++) {
                IloExpr sumSi_k(env);
                for (int k = 0; k < m; k++) {
                    sumSi_k += Si_k[i][k];
                }
                model.add(sumSi_k == 1);
                sumSi_k.end();
            }

            // Constraint 2: R0 + sum(C0,k * S0,k) <= F0
            IloExpr sumC0_k_S0_k(env);
            for (int k = 0; k < m; k++) {
                sumC0_k_S0_k += Ci_k[0][k] * Si_k[0][k];
            }
            model.add(Ri[0] + sumC0_k_S0_k <= Fi[0]);
            sumC0_k_S0_k.end();

            // Constraint 3: max{Ri, Fi-1} + sum(Ci,k * Si,k) <= Fi for i = 1, ..., n-1
            for (int i = 1; i < n; i++) {
                IloExpr sumCi_k_Si_k(env);
                for (int k = 0; k < m; k++) {
                    sumCi_k_Si_k += Ci_k[i][k] * Si_k[i][k];
                }
                model.add(IloMax(Ri[i], Fi[i-1]) + sumCi_k_Si_k <= Fi[i]);
                sumCi_k_Si_k.end();
            }

            // Constraint 4: Fi <= Di for i = 0, ..., n-1
            for (int i = 0; i < n; i++) {
                model.add(Fi[i] <= Di[i]);
            }
        } catch (IloException& e) {
            std::cerr << "Error adding constraints: " << e << std::endl;
            throw; 
        } catch (...) {
            std::cerr << "Unknown error during constraint addition" << std::endl;
            throw; 
        }
    }

    void addObjective() {
        try {
            IloExpr objective(env);
            for (int i = 0; i < n; i++) {
                for (int k = 0; k < m; k++) {
                    objective += Ei_k[i][k] * Si_k[i][k];
                }
            }
            model.add(IloMinimize(env, objective));
            objective.end();
        } catch (IloException& e) {
            std::cerr << "Error adding objective: " << e << std::endl;
            throw; 
        } catch (...) {
            std::cerr << "Unknown error during objective addition" << std::endl;
            throw; 
        }
    }
};

#endif 
