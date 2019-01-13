#ifndef __ABSTRACTED_TASK_H__
#define __ABSTRACTED_TASK_H__

#include <cstdint>
#include <string>
#include <vector>

class AbstractedEcu;
class AbstractedJob;

class AbstractedTask
{
public:
	const std::string            name_; // task name
	const std::uint64_t          phi_; // offset 
	const std::uint64_t          p_; // period 
	const std::uint64_t          c_best_; // best execution time 
	const std::uint64_t          c_worst_; // worst execution time
	const std::uint64_t          memory_usage_;
	const bool                   physical_read_constraint_;
	const bool                   physical_write_constraint_;
	AbstractedEcu* const         mapped_ecu_;
	std::vector<AbstractedTask*> producer_tasks_;
	std::vector<AbstractedTask*> consumer_tasks_;
	std::vector<AbstractedJob*>  pended_offline_guider_jobs_;
	std::vector<AbstractedJob*>  pended_ojpg_jobs_;
	std::vector<AbstractedJob*>  pended_sparse_graph_jobs_;
	std::vector<AbstractedJob*>  pended_dense_graph_jobs_;
	std::vector<AbstractedJob*>  pended_completed_ojpg_jobs_; // exeuction completed 

	AbstractedTask(
		const std::string&   name,
		const std::uint64_t& phi,
		const std::uint64_t& p,
		const std::uint64_t& c_best,
		const std::uint64_t& c_worst,
		const std::uint64_t& memory_usage_,
		const bool&				   physical_read_constraint,
		const bool&          physical_write_constriant,
		AbstractedEcu* const mapped_ecu);
	AbstractedTask(
		const AbstractedTask& ref);
	~AbstractedTask(void);

};

#endif