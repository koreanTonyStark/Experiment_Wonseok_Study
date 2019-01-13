#ifndef __ABSTRACTED_JOB_H__
#define __ABSTRACTED_JOB_H__

#include <string>
#include <cstdint>
#include <vector>
#include <set>
#include <map>

class AbstractedTask;

class AbstractedJob
{
public:
	const std::string          name_;
	AbstractedTask* const      mapped_task_;
	const std::uint64_t        which_period_;

	std::uint64_t              e_real_; // execution time on ECU
	std::uint64_t              e_sim_; // execution time on PC
	std::uint64_t              executed_time_;

	std::uint64_t              t_r_real_; // release time 
	std::uint64_t              t_d_sim_; // deadline

	std::uint64_t              min_t_s_real_;
	std::uint64_t              max_t_s_real_;
	std::uint64_t              min_t_f_real_;
	std::uint64_t              max_t_f_real_;
	
	std::uint64_t              epst_;
	std::uint64_t              lpft_;

	std::set<AbstractedJob*>   j_prev_det_predecessors_;
	std::set<AbstractedJob*>   j_prev_det_successors_;

	//start time set 
	std::set<AbstractedJob*>   j_s_;
	std::set<AbstractedJob*>   j_s_det_predecessors_;
	std::set<AbstractedJob*>   j_s_det_successors_;
	std::set<AbstractedJob*>   j_s_nodet_predecessors_;
	std::set<AbstractedJob*>   j_s_nodet_successors_;

	//finish time set
	std::set<AbstractedJob*>   j_f_;
	std::set<AbstractedJob*>   j_f_det_predecessors_;
	std::set<AbstractedJob*>   j_f_det_successors_;
	std::set<AbstractedJob*>   j_f_nodet_predecessors_;
	std::set<AbstractedJob*>   j_f_nodet_successors_;

	//
	std::set<AbstractedJob*>   j_p_;
	std::set<AbstractedJob*>   j_p_det_predecessors_;
	std::set<AbstractedJob*>   j_p_det_successors_;
	std::set<AbstractedJob*>   j_p_nodet_predecessors_;
	std::set<AbstractedJob*>   j_p_nodet_successors_;

	std::map<AbstractedJob*, std::uint64_t> edge_weights_;

	AbstractedJob(
		const std::string& name,
		AbstractedTask* const mapped_task,
		const std::uint64_t& which_period);
	AbstractedJob(
		const AbstractedJob& ref);
	~AbstractedJob(void);

	static std::uint64_t get_job_index(
		const std::string job_name);

	static std::string assemble_job_name(
		const std::string job_name,
		const std::uint64_t job_index);


};

#endif