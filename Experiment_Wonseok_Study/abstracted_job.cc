#include <algorithm>
#include <limits>

#include "abstracted_job.h"
#include "abstracted_task.h"

AbstractedJob::AbstractedJob(
	const std::string& name,
	AbstractedTask* const mapped_task,
	const std::uint64_t& which_period)
	:
	name_(name),
	mapped_task_(mapped_task),
	which_period_(which_period)
{
	e_real_ = std::numeric_limits<uint64_t>::max();
	e_sim_ = std::numeric_limits<uint64_t>::max();
}

AbstractedJob::AbstractedJob(
	const AbstractedJob& ref)
	:
	name_(ref.name_),
	mapped_task_(ref.mapped_task_),
	which_period_(ref.which_period_),

	e_real_(ref.e_real_),
	e_sim_(ref.e_sim_),
	executed_time_(ref.executed_time_),

	//earliest possible start time
	epst_(ref.epst_),
	//latest possible finish time
	lpft_(ref.lpft_),

	j_prev_det_predecessors_(ref.j_prev_det_predecessors_),
	j_prev_det_successors_(ref.j_prev_det_successors_),

	j_s_(ref.j_s_),
	j_s_det_predecessors_(ref.j_s_det_predecessors_),
	j_s_det_successors_(ref.j_s_det_successors_),
	j_s_nodet_predecessors_(ref.j_s_nodet_predecessors_),
	j_s_nodet_successors_(ref.j_s_nodet_successors_),

	j_f_(ref.j_f_),
	j_f_det_predecessors_(ref.j_f_det_predecessors_),
	j_f_det_successors_(ref.j_f_det_successors_),
	j_f_nodet_predecessors_(ref.j_f_nodet_predecessors_),
	j_f_nodet_successors_(ref.j_f_nodet_successors_),

	j_p_(ref.j_p_),
	j_p_det_predecessors_(ref.j_p_det_predecessors_),
	j_p_det_successors_(ref.j_p_det_successors_),
	j_p_nodet_predecessors_(ref.j_p_nodet_predecessors_),
	j_p_nodet_successors_(ref.j_p_nodet_successors_),
	
	edge_weights_(ref.edge_weights_)
{

}

//return the index of current job 
std::uint64_t AbstractedJob::get_job_index(
	const std::string job_name)
{
	std::size_t s = job_name.find("[");
	std::size_t e = job_name.find("]");
	std::uint64_t ret = std::stoll(job_name.substr(s + 1, e));

	return ret;
}
// format : T1[1]
std::string AbstractedJob::assemble_job_name(
	const std::string job_name,
	const std::uint64_t job_index)
{
	std::size_t s = job_name.find("[");
	std::size_t e = job_name.find("]");

	std::string ret = job_name.substr(0, s + 1) + std::to_string(job_index) + job_name.substr(e);
	
	return ret;
}


