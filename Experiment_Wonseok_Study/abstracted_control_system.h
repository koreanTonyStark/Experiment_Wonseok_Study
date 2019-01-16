#ifndef __ABSTRACTED__CONTROL__SYSTEM__H__
#define __ABSTRACTED__CONTROL__SYSTEM__H__

#include <vector>
#include <string>
#include <map>
#include <set>
#include <cstddef>
#include <stddef.h>

#include "abstracted_ecu.h"
#include "abstracted_task.h"
#include "abstracted_job.h"

typedef std::vector<std::string> time_window_t;

class AbstractedControlSystem
{
public:
	
	//class variables
	const static std::uint64_t heuristics_suf = 0;
	const static std::uint64_t heuristics_wff = 1;
	const static std::uint64_t heuristics_sbf = 2;

	//constructor
	AbstractedControlSystem(void);
	AbstractedControlSystem(const char *system_configuration_file);
	
	//de-constructor
	~AbstractedControlSystem(void);

	//function 
	void ready_offline_guider(void);
	void ready_partitioned_edf(std::uint64_t heuristics, std::uint64_t num_of_cores);
	void run_online_progressive_scheduling(std::uint64_t num_of_hyper_periods);

	//debug
	void print_offline_guider(void);
	void print_ojpg(void);
	void print_simulation_result(void);

private:
	std::vector<AbstractedEcu*>					    ecus_;
	std::vector<AbstractedTask*>				    tasks_;
	std::vector<AbstractedJob*>					    offline_guider_;
	std::vector<AbstractedJob*>					    sparse_graph_;
	std::vector<AbstractedJob*>					    dense_graph_;
	std::vector<AbstractedJob*>					    ojpg_;
	std::vector<AbstractedJob*>				      completed_ojpg_;	

	std::uint64_t		 	   	  						    hyper_period_;
	
	std::map<std::string, time_window_t>    best_case_execution_window_;
	std::map<std::string, time_window_t>    worst_case_execution_window_;

	std::uint64_t                           simulator_num_of_cores_;
	std::vector<std::set<AbstractedJob*>>   simulator_ready_queues_;
	std::vector<std::set<std::string>>      simulator_core_task_mappings_;
	std::map<std::string, std::uint64_t>    simulator_task_core_mappings_;
	std::vector<time_window_t>              simulator_simulation_window_;
	std::uint64_t                           simulator_performance_ratio_;

	void spawn_offline_guider_jobs(void);

	void calculate_execution_windows(void);

	void calculate_start_time_set(
		AbstractedJob* job);
	void partition_start_time_set(
		AbstractedJob* job);

	void calculate_finish_time_set(
		AbstractedJob* job);
	void partition_finish_time_set(
		AbstractedJob* job);
	
	void calculate_producer_time_set(
		AbstractedJob* job);
	void partition_producer_time_set(
		AbstractedJob* job);

	void topological_sort(
		AbstractedJob *root,
		std::vector<AbstractedJob*>& stack,
		std::map<AbstractedJob*, bool>& visited);

	AbstractedJob* initialize_sparse_graph(void);
	void assign_epst(
		AbstractedJob* virtual_start_job);

	AbstractedJob* initialize_dense_graph(void);
	void eliminate_cycles(void);
	AbstractedJob* dfs_dense_graph(
		AbstractedJob* root,
		std::vector<AbstractedJob*>& stack,
		std::map<AbstractedJob*, bool>& visited);
	void assign_lpft(
		AbstractedJob* virtual_start_job);

	void initialize_ojpg(void);

	void assign_ojpg_deadlines(void);

	void find_jobs_to_be_added(
		std::uint64_t current_tick,
		std::vector<AbstractedJob*>& jobs_to_be_added);

	void add_jobs_to_ready_queue(
		std::vector<AbstractedJob*>& jobs_to_be_added);

	void progress_one_tick(
		std::uint64_t current_tick,
		std::vector<AbstractedJob*>& completed_jobs);

	void update_ojpg(
		std::vector<AbstractedJob*>& completed_jobs);

	void update_ojpg_add_new_jobs(
		std::vector<AbstractedJob*>& completed_jobs);

	void update_ojpg_copy_links(
		AbstractedJob* ojpg_job,
		AbstractedJob* corresponding_offline_guider_job);
	
	void update_ojpg_narrow_execution_windows(void);

	void update_ojpg_delete_completed_jobs(
		std::vector<AbstractedJob*>& completed_jobs);

	void update_ojpg_resolve_nondeterminism(void);

	std::vector<AbstractedEcu*>::iterator get_ecu_by_name(
		const std::string ecu_name);
	
	std::vector<AbstractedTask*>::iterator get_task_by_name(
		const std::string task_name);
	std::vector<AbstractedJob*>::iterator get_offline_guider_job_by_name(
		const std::string job_name);
	std::vector<AbstractedJob*>::iterator get_ojpg_job_by_name(
		const std::string job_name);
	std::vector<AbstractedJob*>::iterator get_sparse_graph_job_by_name(
		const std::string job_name);
	std::vector<AbstractedJob*>::iterator get_dense_graph_job_by_name(
		const std::string job_name);
	

	template<typename T>
	T get_gcd(const T a, const T b);
	//what is number_array?? 
	template<typename T>
	T get_lcm(
		const std::vector<T>& number_array);
	
};



#endif