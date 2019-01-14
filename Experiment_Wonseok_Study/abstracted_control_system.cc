#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdint>
#include <set>
#include <algorithm>
#include <limits>

#include "abstracted_control_system.h"

AbstractedControlSystem::AbstractedControlSystem(void)
{
	// Do nothing
}

AbstractedControlSystem::AbstractedControlSystem(
	const char *system_configuration_file)
{

	//Parse file
	std::ifstream in_file(system_configuration_file);
	if (!in_file.is_open())
	{
		std::cerr << "AbstractedControlSystem::AbstractedControlSystem(system_configuration_file)::";
		std::cerr << "file does not exist" << std::endl;
		exit(-1);

	}
	
	std::uint32_t num_of_ecus;
	std::uint32_t num_of_tasks;

	in_file >> num_of_ecus >> num_of_tasks;

	//Input ECU Information 
	for (std::uint32_t ecu_cnt = 0; ecu_cnt < num_of_ecus; ++ecu_cnt)
	{
		std::string ecu_name;
		std::string ecu_sched_policy;

		in_file >> ecu_name >> ecu_sched_policy;

		ecus_.push_back(new AbstractedEcu(ecu_name, ecu_sched_policy));
	}

	for (std::uint32_t task_cnt = 0; task_cnt < num_of_tasks; ++task_cnt)
	{
		std::string task_name;
		std::string mapped_ecu_name;

		std::uint64_t phi, p, c_best, c_worst;
		
		bool physical_read_constraint, physical_write_constraint;
		std::uint64_t physical_read_constraint_uint, physical_write_constraint_uint;
		std::uint64_t memory_usage;

		in_file >> task_name >> mapped_ecu_name;
		in_file >> phi >> p >> c_best >> c_worst;
		in_file >> memory_usage;
		in_file >> physical_read_constraint_uint >> physical_write_constraint_uint;

		physical_read_constraint = (physical_read_constraint_uint == 1) ? true : false;
		physical_write_constraint = (physical_write_constraint_uint == 1) ? true : false;

		//?? 
		AbstractedEcu *mapped_ecu = *get_ecu_by_name(mapped_ecu_name);
		
		simulator_performance_ratio_ = 3; 

		AbstractedTask* task = new AbstractedTask(
			task_name,
			simulator_performance_ratio_ * phi,
			simulator_performance_ratio_ * p,
			simulator_performance_ratio_ * c_best,
			simulator_performance_ratio_* c_worst,
			memory_usage,
			physical_read_constraint,
			physical_write_constraint,
			mapped_ecu);

		tasks_.push_back(task);
		mapped_ecu->pended_tasks_.push_back(task);
	}

	while (!in_file.eof())
	{
		std::string    producer_task_name, consumer_task_name;
		std::uint32_t  number_of_consumer_tasks;
		
		in_file >> producer_task_name >> number_of_consumer_tasks;
		if (producer_task_name.compare("") == 0)
			break;

		for (std::uint32_t consumer_cnt = 0; consumer_cnt < number_of_consumer_tasks; ++consumer_cnt)
		{
			in_file >> consumer_task_name;
			
			AbstractedTask* producer_task = *(this->get_task_by_name(producer_task_name));
			AbstractedTask* consumer_task = *(this->get_task_by_name(consumer_task_name));

			
			producer_task->consumer_tasks_.push_back(consumer_task);
			consumer_task->producer_tasks_.push_back(producer_task);
		}
	}

	in_file.close();

	std::vector<std::uint64_t> periods;
	for (auto task_it = tasks_.begin(); task_it != tasks_.end(); ++task_it)
		periods.push_back((*task_it)->p_);

	hyper_period_ = get_lcm<std::uint64_t>(periods);

	// Sort task following RM order
	// If two different tasks have the same period, then priorities will be assigned following lexicographic order of their task names
	for (auto ecu_it = ecus_.begin(); ecu_it != ecus_.end(); ++ecu_it)
	{
		std::sort((*ecu_it)->pended_tasks_.begin(), (*ecu_it)->pended_tasks_.end(), [](const AbstractedTask *lv, const AbstractedTask* rv)
		{
			if (lv->p_ != rv->p_)
				return lv->p_ < rv->p_;
			else
				//lexicographic order로 return 
				return lv->name_.compare(rv->name_) < 0 ;
		});
	}

}

AbstractedControlSystem::~AbstractedControlSystem(void)
{
	for (auto ecu_it = ecus_.begin(); ecu_it != ecus_.end(); ++ecu_it)
		delete (*ecu_it);
	
	for (auto task_it = tasks_.begin(); task_it != tasks_.end(); ++task_it)
		delete (*task_it);
	
	for (auto offline_guider_it = offline_guider_.begin(); offline_guider_it != offline_guider_.end(); ++offline_guider_it)
		delete (*offline_guider_it);

	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
		delete (*ojpg_it);
	
	for (auto completed_ojpg_it = completed_ojpg_.begin(); completed_ojpg_it != completed_ojpg_.end(); completed_ojpg_it++)
		delete (*completed_ojpg_it);

	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); sparse_graph_it++)
		delete (*sparse_graph_it);

	for (auto dense_graph_it = dense_graph_.begin(); dense_graph_it != dense_graph_.end(); dense_graph_it++)
		delete (*dense_graph_it);
}

/*
 * @ brief Ready offline guider of this control system 
 */

void AbstractedControlSystem::ready_offline_guider(void)
{
	//1.14
	spawn_offline_guider_jobs();
	calculate_execution_windows();
	
	// Since hat (^) jobs will added to the jobs_ while offline guider is building,
	// Offine guider building should be started with captured jobs
	//offline guider에 존재하는 job의 vector 
	std::vector<AbstractedJob*> captured_offline_guider(offline_guider_.begin(), offline_guider_.end());

	for (auto offline_guider_it = captured_offline_guider.begin(); offline_guider_it != captured_offline_guider.end(); ++offline_guider_it)
	{
		if ((*offline_guider_it)->mapped_task_->physical_read_constraint_ == true)
		{
			calculate_start_time_set((*offline_guider_it));
			partition_start_time_set((*offline_guider_it));
		}
		if ((*offline_guider_it)->mapped_task_->physical_write_constraint_ == true)
		{
			calculate_finish_time_set((*offline_guider_it));
			partition_finish_time_set((*offline_guider_it));
		}
		if ((*offline_guider_it)->mapped_task_->producer_tasks_.empty() == false)
		{
			calculate_producer_time_set((*offline_guider_it));
			partition_producer_time_set((*offline_guider_it));
		}

	}
	
}

/* 
 * @brief Spwan 2 * (H.P / Pi) jobs for each task Ti and connect deterministic edges
 *			  between two consectuvie jos of task Ti 
 * //같은 task가 spawn한 job들은 반드시 deterministic edge를 가지므로 연결 
 */
void AbstractedControlSystem::spawn_offline_guider_jobs(void)
{
	for (auto task_it = tasks_.begin(); task_it != tasks_.end(); task_it++)
	{
		std::uint64_t job_idx = 0;
		AbstractedJob* prev_offline_guider_job = NULL;
		
		for (std::uint64_t tick = (*task_it)->phi_; tick < 2 * hyper_period_; tick += (*task_it)->p_)
		{
			std::string offline_guider_job_name = (*task_it)->name_ + "[" + std::to_string(job_idx++) + "]";

			AbstractedJob* offline_guider_job = new AbstractedJob(offline_guider_job_name, (*task_it), tick / hyper_period_);
			offline_guider_job->t_r_real_ = tick;
			offline_guider_.push_back(offline_guider_job);
			(*task_it)->pended_offline_guider_jobs_.push_back(offline_guider_job);
			if (prev_offline_guider_job != NULL)
			{
				offline_guider_job->j_prev_det_predecessors_.insert(prev_offline_guider_job);
				prev_offline_guider_job->j_prev_det_successors_.insert(offline_guider_job);
			}
			prev_offline_guider_job = offline_guider_job;
		}
	}
}

/*
 * @ brief Calculate best/worst-case execution windows and assign earliest/latest
 *         start-time/finish-time ranges for each spawned jobs
 *
 */

void AbstractedControlSystem::calculate_execution_windows(void)
{
	for (auto ecu_it = ecus_.begin(); ecu_it != ecus_.end(); ++ecu_it)
	{
		auto filler = [&](std::map<std::string, time_window_t>& window, const std::uint8_t mode)
		{

			//time_window_t vector<string> type, 2 * HP size의 각각의 값에 대해서 string 값을 가짐 
			window[(*ecu_it)->name_] = time_window_t(2 * hyper_period_, "EMPTY");

			for (auto task_it = (*ecu_it)->pended_tasks_.begin(); task_it != (*ecu_it)->pended_tasks_.end(); ++task_it)
			{
				std::uint64_t job_idx = 0 ;

				for (std::uint64_t tick = (*task_it)->phi_; tick < 2 * hyper_period_; tick += (*task_it)->p_)
				{
					//tick은 offset부터 시작해서 주기만큼 증가 
					std::uint64_t executed_time = 0;
					std::uint64_t current_time = tick;
					

					std::string     job_name = (*task_it)->name_ + "[" + std::to_string(job_idx) + "]";
					std::uint64_t   job_execution_time = mode == 0 ? (*task_it)->c_best_ : (*task_it)->c_worst_;
					//job_pointer하나 선언해서 offline_guider에서 해당 job을 찾는다 
					AbstractedJob*  job_pointer = *get_offline_guider_job_by_name(job_name);


					while (executed_time < job_execution_time)
					{
						//empty인 경우(priority 조절해서 pended_tasks_에 들어가있는가?  
						if (window[(*ecu_it)->name_][current_time].compare("EMPTY") == 0)
						{
							if (executed_time == 0)
								(mode == 0 ? job_pointer->min_t_s_real_ : job_pointer->max_t_s_real_) = current_time;
							
							window[(*ecu_it)->name_][current_time] = job_name;
							++executed_time;
						}
						++current_time;
					}
					(mode == 0 ? job_pointer->min_t_f_real_: job_pointer->max_t_f_real_) = current_time;
				}
			}
		};

		//minimum start time + minimum finish time 
		filler(best_case_execution_window_, 0);

		//maximum start time + maximum finish time 
		filler(worse_case_execution_window_, 1);
	}
}

/*
 * @brief  Calculate start-time set of given job
 *
 * @param  job : The pointer of job whose start-time set is calculated 
 */

void AbstractedControlSystem::calculate_start_time_set(
	AbstractedJob* job)
{
	//worst-case busy period start-time 
	std::uint64_t wcbp_start_real;
	std::uint64_t max_t_s_real;

	const AbstractedEcu* mapped_ecu = job->mapped_task_->mapped_ecu_;

	std::int64_t wcbp_start_real_it;
	for (wcbp_start_real_it = job->max_t_f_real_ - 1; wcbp_start_real_it >= 0; --wcbp_start_real_it)
	{
		//"EMPTY"인 곳 찾으면 거기는 busy-period안에 속하지 X, busy-period : 내가 release되고 나보다 priority 높은 job이나 내가 수행될때까지의 시간 
		if (worse_case_execution_window_[mapped_ecu->name_][wcbp_start_real_it].compare("EMPTY") == 0)
			break;
	}
	//empty인 곳이 발견 되면 worst-case busy period가 
	wcbp_start_real = ++wcbp_start_real_it;

	//If it is second HP job then WCBP cannot span over previous period
	//which_period_ = 1 이면 HP ~ 2HP 사이의 job을 의미 
	if (job->which_period_ == 1)
		wcbp_start_real = std::max<std::int64_t>(wcbp_start_real, hyper_period_);

	for (auto task_it = mapped_ecu->pended_tasks_.begin(); task_it != mapped_ecu->pended_tasks_.end(); ++task_it)
	{
		// priority낮은 task들의 job은 아예 배제 
		// Skip lower or priority tasks 
		// If two different tasks have the same period, then priorities will be assigned following lexicographic order of their task names
		if ((*task_it)->p_ > job->mapped_task_->p_)
			continue;
		else if ((*task_it)->p_ == job->mapped_task_->p_ && (*task_it)->name_.compare(job->mapped_task_->name_) >= 0) // lower priority task 
			continue;

		// Find start-time set에 걸리는 job들을 찾아서 넣는다 
		for (auto job_it = (*task_it)->pended_offline_guider_jobs_.begin(); job_it != (*task_it)->pended_offline_guider_jobs_.end(); ++job_it)
		{
			//worst-case busy period 보다 release되는 시작이 크거나 같고, release되는 시간이 내 job의 maximum start time보다는 작을 때
			if (wcbp_start_real <= (*job_it)->t_r_real_ && (*job_it)->t_r_real_ < max_t_s_real)
				job->j_s_.insert((*job_it));
		}
	}
}

/*
 * @brief  Partition start-time set of given job by deterministic and non-deterministic predecessors
 *
 * @param  job : The pointer of job whose start-time set is partitioned
 */
void AbstractedControlSystem::partition_start_time_set(
	AbstractedJob* job)
{
	for (auto job_it = job->j_s_.begin(); job_it != job->j_s_.end(); ++job_it)
	{
		
		//deterministic edge 
		//내 job의 minimum start time이 다른job의 maxixmum start time보다 크면, 당연히 다른 job이 우선시 된다 -> deterministic 
		//start time set의 predecessor, successor 집합은 c++ stl set으로 구현(중복 거르려고)
		if ((*job_it)->max_t_s_real_ < job->min_t_s_real_)
		{
			job->j_s_det_predecessors_.insert((*job_it));
			(*job_it)->j_s_det_successors_.insert(job);
		}
		//non-demeterministic edge 
		else
		{
			job->j_s_nodet_predecessors_.insert((*job_it));
			(*job_it)->j_s_nodet_successors_.insert(job);
		}
	}
}


