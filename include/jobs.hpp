#ifndef JOBS_HPP
#define JOBS_HPP

#include <ostream>
#include <vector>
#include <algorithm> // for find
#include <functional> // for hash
#include <exception>
#include <cmath>

#include "time.hpp"
#include "interval.hpp"

namespace NP {

	typedef std::size_t hash_value_t;
	

	struct JobID {
		unsigned long job;
		unsigned long task;

		JobID(unsigned long j_id, unsigned long t_id)
		: job(j_id), task(t_id)
		{
		}

		bool operator==(const JobID& other) const
		{
			return this->task == other.task && this->job == other.job;
		}

		friend std::ostream& operator<< (std::ostream& stream, const JobID& id)
		{
			stream << "T" << id.task << "J" << id.job;
			return stream;
		}
	};

	template<class Time> class Job {

	public:
		typedef std::vector<Job<Time>> Job_set;
		typedef Time Priority; // Make it a time value to support EDF
		typedef std::vector<float> Speed_space; // Set of valid speeds for the job (Energy aware speed scaling)
		typedef float Energy;
		
		

	private:
		Interval<Time> arrival;
		Interval<Time> cost; 
		Interval<Time> high_speed_cost; 
		Time deadline;
		Priority priority;
		JobID id;
		hash_value_t key;
		Speed_space speed;
		Energy consumption;


		void compute_hash() {
			auto h = std::hash<Time>{};
			key = h(arrival.from());
			key = (key << 4) ^ h(id.task);
			key = (key << 4) ^ h(arrival.until());
			key = (key << 4) ^ h(cost.from());
			key = (key << 4) ^ h(deadline);
			key = (key << 4) ^ h(cost.upto());
			key = (key << 4) ^ h(id.job);
			key = (key << 4) ^ h(priority);
		}

	public:

		Job(unsigned long id,
			Interval<Time> arr, Interval<Time> cost,
			Time dl, Priority prio,
			unsigned long tid = 0)
		: arrival(arr), cost(cost),
		  deadline(dl), priority(prio), id(id, tid), high_speed_cost(cost)
		{
			compute_hash();
		}

		Speed_space get_speed_space()
		{
			return speed;
		}

		Interval<Time> get_ultimate_finish_time(Interval<Time> state_ultimate_start_time ){
			/*
			Calculate ultimate finish times for given ultimate start time interval
			based on the available speed 
			*/
			Interval<Time> ultimate_time;
			// Add floor least cost at highest speed
			ultimate_time.a = state_ultimate_start_time.a + max(floor(least_cost() /speed.back()),1);
			// Add ceil maximal cost at lowest speed
			ultimate_time.b = state_ultimate_start_time.b + ceil(maximal_cost()/speed.front());
			return ultimate_time;
		}

		void update_speed_space(Speed_space input_speed_space)
		{
			speed = input_speed_space;
			double a = std::max(1.0,floor(high_speed_cost.from()/speed.front()));
			double b = ceil(high_speed_cost.upto()/speed.front());
			cost = Interval<Time>(a,b);
			consumption = calculate_energy(5-speed.size())*cost.until(); // 5 is const needs better abstraction
		} 

		Energy get_energy() const
		{
			
			return speed.empty() ? 0 : consumption ; // If no speed then empty
			
		}

		Energy calculate_energy(size_t index)
		{
			// TODO: remove hardcoded dvfs values and provide everything in yaml file
			// Energy equation and values based on Samsung Exynos 4210 Processor A9 core
			
			// std::cout << "index is " << index <<std::endl;
			const std::vector<std::vector<float>> dvfs_setting =
			{
				{1.0,1.0327},
				{1.05,1.12870},
				{1.10,1.2218},
				{1.15,1.3122},
				{1.2,1.4}
			};
			float vcpu = dvfs_setting[index][0];
			float fcpu = dvfs_setting[index][1];
			return (0.4*((0.446*vcpu*vcpu*fcpu)+(0.1793*vcpu)- 0.1527)); // assumption that only one of the core is contributing to the current job
		}



		void set_cost_to_ultimate()
		{
			double a = std::max(1.0,floor(high_speed_cost.from()/speed.back()));
			double b = ceil(high_speed_cost.upto()/speed.front());
			cost = Interval<Time>(a,b);
		}		

		hash_value_t get_key() const
		{
			return key;
		}

		Time earliest_arrival() const
		{
			return arrival.from();
		}

		Time latest_arrival() const
		{
			return arrival.until();
		}

		const Interval<Time>& arrival_window() const
		{
			return arrival;
		}

		Time least_cost() const
		{
			return cost.from();
		}

		Time maximal_cost() const
		{
			return cost.upto();
		}

		const Interval<Time>& get_cost() const
		{
			return cost;
		}

		Priority get_priority() const
		{
			return priority;
		}

		Time get_deadline() const
		{
			return deadline;
		}

		bool exceeds_deadline(Time t) const
		{
			return t > deadline
			       && (t - deadline) >
			          Time_model::constants<Time>::deadline_miss_tolerance();
		}

		JobID get_id() const
		{
			return id;
		}

		unsigned long get_job_id() const
		{
			return id.job;
		}

		unsigned long get_task_id() const
		{
			return id.task;
		}

		bool is(const JobID& search_id) const
		{
			return this->id == search_id;
		}

		bool higher_priority_than(const Job &other) const
		{
			return priority < other.priority
			       // first tie-break by task ID
			       || (priority == other.priority
			           && id.task < other.id.task)
			       // second, tie-break by job ID
			       || (priority == other.priority
			           && id.task == other.id.task
			           && id.job < other.id.job);
		}

		bool priority_at_least_that_of(const Job &other) const
		{
			return priority <= other.priority;
		}

		bool priority_exceeds(Priority prio_level) const
		{
			return priority < prio_level;
		}

		bool priority_at_least(Priority prio_level) const
		{
			return priority <= prio_level;
		}

		Interval<Time> scheduling_window() const
		{
			// inclusive interval, so take off one epsilon
			return Interval<Time>{
			                earliest_arrival(),
			                deadline - Time_model::constants<Time>::epsilon()};
		}

		static Interval<Time> scheduling_window(const Job& j)
		{
			return j.scheduling_window();
		}

		friend std::ostream& operator<< (std::ostream& stream, const Job& j)
		{
			stream << "Job{" << j.id.job << ", " << j.arrival << ", "
			       << j.cost << ", " << j.deadline << ", " << j.priority
			       << ", " << j.id.task << "}";
			return stream;
		}

	};

	template<class Time>
	bool contains_job_with_id(const typename Job<Time>::Job_set& jobs,
	                          const JobID& id)
	{
		auto pos = std::find_if(jobs.begin(), jobs.end(),
		                        [id] (const Job<Time>& j) { return j.is(id); } );
		return pos != jobs.end();
	}

	class InvalidJobReference : public std::exception
	{
		public:

		InvalidJobReference(const JobID& bad_id)
		: ref(bad_id)
		{}

		const JobID ref;

		virtual const char* what() const noexcept override
		{
			return "invalid job reference";
		}

	};

	template<class Time>
	const Job<Time>& lookup(const typename Job<Time>::Job_set& jobs,
	                                 const JobID& id)
	{
		auto pos = std::find_if(jobs.begin(), jobs.end(),
		                        [id] (const Job<Time>& j) { return j.is(id); } );
		if (pos == jobs.end())
			throw InvalidJobReference(id);
		return *pos;
	}

	// template<class Time>
	// Job<Time>& lookup( typename Job<Time>::Job_set& jobs,
	//                                 JobID& id)
	// {
	// 	auto pos = std::find_if(jobs.begin(), jobs.end(),
	// 	                        [id] (const Job<Time>& j) { return j.is(id); } );
	// 	if (pos == jobs.end())
	// 		throw InvalidJobReference(id);
	// 	return *pos;
	// }

}

namespace std {
	template<class T> struct hash<NP::Job<T>>
	{
		std::size_t operator()(NP::Job<T> const& j) const
		{
			return j.get_key();
		}
	};

	template<> struct hash<NP::JobID>
	{
		std::size_t operator()(NP::JobID const& id) const
		{
			hash<unsigned long> h;
			return (h(id.job) << 4) ^ h(id.task);
		}

	};
}

#endif
