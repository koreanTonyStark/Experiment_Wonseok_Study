#include <algorithm>

#include "abstracted_task.h"
#include "abstracted_ecu.h"
#include "abstracted_job.h"

AbstractedTask::AbstractedTask(
	const std::string&   name,
	const std::uint64_t& phi,
	const std::uint64_t& p,
	const std::uint64_t& c_best,
	const std::uint64_t& c_worst,
	const std::uint64_t& memory_usage,
	const bool&          physical_read_constraint,
	const bool&          physical_write_constraint,
	AbstractedEcu* const mapped_ecu)
	:
	name_(name),
	phi_(phi),
	p_(p),
	c_best_(c_best),
	c_worst_(c_worst),
	memory_usage_(memory_usage_),
	physical_read_constraint_(physical_read_constraint),
	physical_write_constraint_(physical_write_constraint),
	mapped_ecu_(mapped_ecu)
{

}

AbstractedTask::AbstractedTask(
	const AbstractedTask& ref)
	:
	name_(ref.name_),
	phi_(ref.phi_),
	p_(ref.p_),
	c_best_(ref.c_best_),
	c_worst_(ref.c_worst_),
	memory_usage_(ref.memory_usage_),
	physical_read_constraint_(ref.physical_read_constraint_),
	physical_write_constraint_(ref.physical_write_constraint_),
	mapped_ecu_(ref.mapped_ecu_)
{
	pended_offline_guider_jobs_.resize(ref.pended_offline_guider_jobs_.size());
	std::copy(
		ref.pended_offline_guider_jobs_.begin(), 
		ref.pended_offline_guider_jobs_.end(), 
		pended_offline_guider_jobs_.begin());

	pended_ojpg_jobs_.resize(ref.pended_ojpg_jobs_.size());
	std::copy(ref.pended_ojpg_jobs_.begin(), ref.pended_ojpg_jobs_.end(), pended_ojpg_jobs_.begin());

	pended_sparse_graph_jobs_.resize(ref.pended_sparse_graph_jobs_.size());
	std::copy(ref.pended_sparse_graph_jobs_.begin(), ref.pended_sparse_graph_jobs_.end(), pended_sparse_graph_jobs_.begin());

	pended_dense_graph_jobs_.resize(ref.pended_dense_graph_jobs_.size());
	std::copy(ref.pended_dense_graph_jobs_.begin(), ref.pended_dense_graph_jobs_.end(), pended_dense_graph_jobs_.begin());

	pended_completed_ojpg_jobs_.resize(ref.pended_completed_ojpg_jobs_.size());
	std::copy(ref.pended_completed_ojpg_jobs_.begin(), ref.pended_completed_ojpg_jobs_.end(), pended_completed_ojpg_jobs_.begin());

	producer_tasks_.resize(ref.producer_tasks_.size());
	std::copy(ref.producer_tasks_.begin(), ref.producer_tasks_.end(), producer_tasks_.begin());

	consumer_tasks_.resize(ref.consumer_tasks_.size());
	std::copy(ref.consumer_tasks_.begin(), ref.consumer_tasks_.end(), consumer_tasks_.begin());


}

AbstractedTask::~AbstractedTask(void)
{
	//Do nothing
}
	