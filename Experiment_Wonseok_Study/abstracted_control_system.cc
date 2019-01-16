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
 * @brief Run online-progressive scheduling assuming multi-core usage
 *				based on Kyoung-Soo We and Wonseok Lee's approach
 *
 * @param num_of_hyper_periods Online-progressive scheduling algorithm will
 *														 simulate the whole system during the num_of_hyper_periods * HP
 *
 */
void AbstractedControlSystem::run_online_progressive_scheduling(
	std::uint64_t num_of_hyper_periods)
{
	// Progress Simulation until num_of_hyper_period * hyper_period_
	for (std::uint64_t synthesized_tick = 0; synthesized_tick < num_of_hyper_periods*hyper_period_; ++synthesized_tick)
	{
		std::vector<AbstractedJob*> jobs_to_be_added;
		
		find_jobs_to_be_added(synthesized_tick, jobs_to_be_added);

		add_jobs_to_ready_queue(jobs_to_be_added);

		std::vector<AbstractedJob*> completed_jobs;

		progress_one_tick(synthesized_tick, completed_jobs);

		if (completed_jobs.empty())
			continue;

		update_ojpg(completed_jobs);

	}

	std::cout << "Simulatable (E:" << ecus_.size() << ", T:" << tasks_.size() << ")" << std::endl;
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
		filler(worst_case_execution_window_, 1);
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
		if (worst_case_execution_window_[mapped_ecu->name_][wcbp_start_real_it].compare("EMPTY") == 0)
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

/* 
 * @brief Calculate finish-time set of given job
 *		    
 * @param job : The pointer of job whose finish-time set is calculated
 */

void AbstractedControlSystem::calculate_finish_time_set(
	AbstractedJob* job)
{
	std::uint64_t wcbp_start_real;
	std::uint64_t max_t_f_real;

	const AbstractedEcu *mapped_ecu = job->mapped_task_->mapped_ecu_;
	
	std::int64_t wcbp_start_real_it;
	for (wcbp_start_real_it = job->max_t_f_real_ - 1; wcbp_start_real_it >= 0; --wcbp_start_real_it)
	{
		if (worst_case_execution_window_[mapped_ecu->name_][wcbp_start_real_it].compare("EMPTY") == 0)
			break;
	}
	wcbp_start_real = ++wcbp_start_real_it;

	if (job->which_period_ == 1)
		wcbp_start_real = std::max<std::uint64_t>(wcbp_start_real, hyper_period_);

	max_t_f_real = job->max_t_f_real_;

	for (auto task_it = mapped_ecu->pended_tasks_.begin(); task_it != mapped_ecu->pended_tasks_.end(); ++task_it)
	{
		// Skip lower priority tasks
		// If two different tasks have the same period, then priorities will be assigned following lexicographic order of their task names
		if ((*task_it)->p_ > job->mapped_task_->p_)
			continue;
		else if ((*task_it)->p_ == job->mapped_task_->p_ && (*task_it)->name_.compare(job->mapped_task_->name_) >= 0)
			continue;

		//Find finish-time set
		for (auto job_it = (*task_it)->pended_offline_guider_jobs_.begin(); job_it != (*task_it)->pended_offline_guider_jobs_.end(); ++job_it)
		{
			//어떤 job의 release time이 worst-case busy period보다는 크고, 우선순위 높은 job의 maximum finish time보다는 작을 때, finish-time set에 insert
			if (wcbp_start_real <= (*job_it)->t_r_real_ && (*job_it)->t_r_real_ < max_t_f_real)
				job->j_f_.insert((*job_it));
		}
	}

	//finish-time set은 자기자신도 들어감 
	job->j_f_.insert(job);

}

/*
 * @brief Partition finish-time set of given job by deterministic and non-deterministic predecessors
 *				hat (^) jobs will be added
 *
 *
 * @param job : The pointer of job whose finish-time set is partitioned
 */

void AbstractedControlSystem::partition_finish_time_set(
	AbstractedJob* job)
{
	AbstractedJob *hat_job = new AbstractedJob(job->name_ + "^", job->mapped_task_, job->which_period_);
	//execution time 0 만들기 위해서 
	hat_job->min_t_s_real_ = job->min_t_f_real_;
	hat_job->max_t_s_real_ = job->min_t_f_real_;
	hat_job->min_t_f_real_ = hat_job->max_t_s_real_;
	hat_job->max_t_f_real_ = hat_job->max_t_s_real_;

	for (auto job_it = job->j_f_.begin(); job_it != job->j_f_.end(); ++job_it)
	{
		//job_it가 우선 실행될게 확실 
		if ((*job_it)->max_t_s_real_ < job->min_t_f_real_)
		{
			hat_job->j_f_det_predecessors_.insert((*job_it));
			(*job_it)->j_f_det_successors_.insert(hat_job);
		}
		else
		{
			hat_job->j_f_nodet_predecessors_.insert((*job_it));
			(*job_it)->j_f_det_successors_.insert(hat_job);
		}

	}
	
	//offline guider에 hat job도 push 
	offline_guider_.push_back(hat_job);
}

/*
 * @brief Calculate producer-time set of given job
 *
 * @param job : The pointer of job whose producer-time set is calculated
 */

void AbstractedControlSystem::calculate_producer_time_set(
	AbstractedJob* job)
{
	//potential producer의 finish time set U its own start-time set
	const AbstractedTask *mapped_task = job->mapped_task_;
	for (auto task_it = mapped_task->producer_tasks_.begin(); task_it != mapped_task->producer_tasks_.end(); ++task_it)
	{
		for (auto job_it = (*task_it)->pended_offline_guider_jobs_.begin(); job_it != (*task_it)->pended_offline_guider_jobs_.end(); ++job_it)
		{
			//Potential producers' finish-time set
			//내 job의 최대시작시간보다 일찍수행이 종료되는 job의 finish time set + 내 job의 최소시작시간보다 최대종료시간이 긴 애들 
			if (job->max_t_s_real_ >= (*job_it)->min_t_f_real_ && job->min_t_s_real_ <= (*job_it)->max_t_f_real_)
			{
				calculate_finish_time_set((*job_it));
				for (auto j_f_it = (*job_it)->j_f_.begin(); j_f_it != (*job_it)->j_f_.end(); ++j_f_it)
					job->j_p_.insert((*j_f_it));
			}
			
		}
	}
	
	//its own start-time set 
	calculate_start_time_set(job);
	for (auto j_s_it = job->j_s_.begin(); j_s_it != job->j_s_.end(); ++j_s_it)
		job->j_p_.insert((*j_s_it));

}

/* 
 * @brief Parition producer-time set of given job by deterministic and non-deterministic predecessors
 *
 * @param job : The pointer of job whose producer-time set is partitioned
 */

void AbstractedControlSystem::partition_producer_time_set(
	AbstractedJob* job)
{
	// The first condition, among the job in J_p of this job (producer-time set안에서 edge det/non-det 나누는 것)
	for (auto job_it = job->j_p_.begin(); job_it != job->j_p_.end(); ++job_it)
	{
		if ((*job_it)->max_t_s_real_ < job->min_t_s_real_)
		{
			job->j_p_det_predecessors_.insert((*job_it));
			(*job_it)->j_p_det_successors_.insert(job);
		}
		else
		{
			job->j_p_nodet_predecessors_.insert((*job_it));
			(*job_it)->j_p_nodet_successors_.insert(job);
		}

	}

	const AbstractedTask *mapped_task = job->mapped_task_;
	for (auto task_it = mapped_task->producer_tasks_.begin(); task_it != mapped_task->producer_tasks_.end(); ++task_it)
	{
		for (auto job_it = (*task_it)->pended_offline_guider_jobs_.begin(); job_it != (*task_it)->pended_offline_guider_jobs_.end(); ++job_it)
		{
			if (job_it + 1 == (*task_it)->pended_offline_guider_jobs_.end())
				break;
			
			if ((*job_it)->max_t_f_real_ <= job->min_t_s_real_ && job->min_t_s_real_ < (*(job_it + 1))->max_t_f_real_)
			{
				job->j_p_det_predecessors_.insert((*job_it));
				(*job_it)->j_p_det_successors_.insert(job);
			}	
		}
	}
}

/* 
 * @brief Find jobs that can be added to ready queues of each core
 *				The jobs which has no un-completed deterministic predecessors
 *				If the jobs has physical-read constraint, then this job should start after its real start time
 *
 * @param current_tick			current_tick
 *        jobs_to_be_added  jobs that can be added to ready queues will be saved here
 */
void AbstractedControlSystem::find_jobs_to_be_added(
	std::uint64_t current_tick,
	std::vector<AbstractedJob*>& jobs_to_be_added)
{
	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
	{
		bool is_there_det_predecessors = false;
		is_there_det_predecessors |= !(*ojpg_it)->j_prev_det_predecessors_.empty();
		is_there_det_predecessors |= !(*ojpg_it)->j_s_det_predecessors_.empty();
		is_there_det_predecessors |= !(*ojpg_it)->j_f_det_predecessors_.empty();
		is_there_det_predecessors |= !(*ojpg_it)->j_p_det_predecessors_.empty();

		if (!is_there_det_predecessors)
		{
			if ((*ojpg_it)->mapped_task_->physical_read_constraint_) // If this job has physical read constraint 
			{
				std::uint64_t t_s_real = (*ojpg_it)->min_t_s_real_;

				if (t_s_real <= current_tick)
					jobs_to_be_added.push_back((*ojpg_it));
			}
			else
				jobs_to_be_added.push_back((*ojpg_it));
		}
	}
}

/**
 *  @brief Add jobs to corresponding ready queues
 *				 At this step, e_real_ and e_sim_ are randomly assigned
 *				 To adjust ECU-PC execution performance ratio you should modify below function
 *
 *  @param jobs_to_be_added jobs to be added
 *
 */
void AbstractedControlSystem::add_jobs_to_ready_queue(
	std::vector<AbstractedJob*>& jobs_to_be_added)
{
	for (auto job_it = jobs_to_be_added.begin(); job_it != jobs_to_be_added.end(); ++job_it)
	{
		std::uint64_t mapped_core = simulator_task_core_mappings_[(*job_it)->mapped_task_->name_];
		//already exist 
		if (simulator_ready_queues_[mapped_core].insert(*job_it).second == false)
			continue;
		
		std::uint64_t bcet = (*job_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
		std::uint64_t wcet = (*job_it)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		
		//if this job is not hat job
		if ((*job_it)->name_.find("^") == std::string::npos)
		{
			//job name의 index가 겹칠 일이 생길 수도?? 
			/* Below for experiment */
			srand((unsigned int)AbstractedJob::get_job_index((*job_it)->name_));
			/* Above for experiment */
			(*job_it)->e_real_ = (bcet + (rand() % (wcet - bcet + 1))) * simulator_performance_ratio_;
			(*job_it)->e_sim_ = (*job_it)->e_real_ / simulator_performance_ratio_;
		}
		else
			(*job_it)->e_real_ = (*job_it)->e_sim_ = 0;

		//not executed yet, just pushed on ready-queue 
		(*job_it)->executed_time_ = 0;
	}
}

/**
 * @brief  Progress simulation as much as one tick
 *				 Each core executes its the highest priority job following EDF order
 *
 *
 * @param current_tick		current tick
 * @param completed_jobs	jobs which is completed on simulator
 */

void AbstractedControlSystem::progress_one_tick(
	std::uint64_t current_tick,
	std::vector<AbstractedJob*>& completed_jobs)
{
	for (std::uint64_t core = 0; core < simulator_num_of_cores_; ++core)
	{
		//매 tick마다 empty를 넣어주는 이유?? 
		simulator_simulation_window_[core].push_back("EMPTY");

		//Hat-jobs should be finished right after it is added to ready queue
		for (auto job_it = simulator_ready_queues_[core].begin(); job_it != simulator_ready_queues_[core].end();)
		{
			if ((*job_it)->name_.find("^") != std::string::npos)
			{
				completed_jobs.push_back((*job_it));
				job_it = simulator_ready_queues_[core].erase(job_it);
			}
			else
				++job_it;
		}
		
		if (simulator_ready_queues_[core].empty())
			continue;

		// Find the highest priority job following EDF order
		AbstractedJob* job_to_be_executed = nullptr;
		for (auto job_it = simulator_ready_queues_[core].begin(); job_it != simulator_ready_queues_[core].end(); ++job_it)
		{
			if (job_to_be_executed == nullptr || (job_to_be_executed->t_d_sim_ > (*job_it)->t_d_sim_))
				job_to_be_executed = (*job_it);
		}
		
		//EMPTY를 job name으로 바꿔준다 
		if (simulator_simulation_window_[core].back().compare("EMPTY") == 0)
			simulator_simulation_window_[core].back() = job_to_be_executed->name_;
		else
			simulator_simulation_window_[core].back() += "," + job_to_be_executed->name_;
		
		//다른 job들은 ++ 안해줌?? (what if simulator_simulation_window의 back에 둘 이상의 job이 존재하면? 
		++job_to_be_executed->executed_time_;
		
		if (job_to_be_executed->executed_time_ == job_to_be_executed->e_sim_)
		{
			completed_jobs.push_back(job_to_be_executed);
			simulator_ready_queues_[core].erase(job_to_be_executed);
		}
	}

	//Deadline miss check
	for (auto completed_job_it = completed_jobs.begin(); completed_job_it != completed_jobs.end(); ++completed_job_it)
	{
		if ((*completed_job_it)->t_d_sim_ < current_tick + 1)
		{
			std::cerr << "AbstractedControlSytem::progress_one_tick(current_tick, completed_jobs)::";
			std::cerr << "deadline miss is detected" << "(" << (*completed_job_it)->name_ << ")" << std::endl;
			std::cout << "Not Simulatable (E: " << ecus_.size() << ", T:" << tasks_.size() << ")::deadline_miss" << std::endl;
			exit(-1);
		}
	}
}

/**
  * @brief	Ready partitioned EDF using below idea (by Wonseok Lee)
	*					- The total memory usage of tasks mapped to a single core
	*						does not exceed the last level cache size of its core
	*					- other heuristics is determined by parameter
	*
	* @param	heuristics Smllest-Utilization-First when it is 0
	*										 Worst-Fit-First when it is 1
	*										 Smallest-Block-First when it is 2
	* @param  num_of_cores Online-progressive scheduling algorithm will
	*											 use num_of_cores CPU cores to simulating the system
	*
	*/
void AbstractedControlSystem::ready_partitioned_edf(
	std::uint64_t heuristics,
	std::uint64_t num_of_cores)
{
	initialize_ojpg();

	//1번째 HP는 버림 
	//Drop the 1st HP's execution windows 
	for (auto ecu_it = ecus_.begin(); ecu_it != ecus_.end(); ++ecu_it)
	{
		//begin에서 hyper_period까지는 버리고 다음 range의 hyper_period_부터로 resize 
		best_case_execution_window_[(*ecu_it)->name_] = time_window_t(
			best_case_execution_window_[(*ecu_it)->name_].begin() + hyper_period_,
			best_case_execution_window_[(*ecu_it)->name_].end());
		worst_case_execution_window_[(*ecu_it)->name_] = time_window_t(
			worst_case_execution_window_[(*ecu_it)->name_].begin() + hyper_period_,
			worst_case_execution_window_[(*ecu_it)->name_].end());
	}

	// Ready partitioned EDF
	simulator_num_of_cores_ = num_of_cores;
	
	//모두 empty한 것으로 initialize해서 push 
	for (std::uint64_t core = 0; core < num_of_cores; ++core)
	{
		
		simulator_ready_queues_.push_back(std::set<AbstractedJob*>());
		simulator_core_task_mappings_.push_back(std::set <std::string>());
	}
	//각 core마다 simulation window를 assign 
	simulator_simulation_window_.assign(simulator_num_of_cores_, time_window_t());
	
	if (heuristics == AbstractedControlSystem::heuristics_suf)
	{
		std::vector<std::uint64_t> simulator_mem_bin(simulator_num_of_cores_, 256); //최대 용량 256kb로 잡음 
		std::vector<double> simulator_util_bin(simulator_num_of_cores_, 0.0); //4개 core의 bin에 처음에 다 utilization 0.0 
		
		for (auto task_it = tasks_.begin(); task_it != tasks_.end(); ++task_it)
		{
			double min_util = std::numeric_limits<double>::max();
			std::uint64_t min_util_core = 0;
			
			for (std::uint64_t core = 0; core < simulator_num_of_cores_; ++core)
			{
				if (simulator_mem_bin[core] >= (*task_it)->memory_usage_ && min_util > simulator_util_bin[core]  )
				{
					min_util = simulator_util_bin[core];
					min_util_core = core;
				}

				if (min_util == std::numeric_limits<double>::max())
				{
					std::cout << "Not Simulatable (E: " << ecus_.size() << ", T:" << tasks_.size() << ")::memory_constraint_violation" << std::endl;
					exit(-1);
				}
			}

			//사용되지 않응 양은 빼고 
			simulator_mem_bin[min_util_core] -= (*task_it)->memory_usage_;
			//사용된 양은 더한다 
			simulator_util_bin[min_util_core] += (double)((*task_it)->c_worst_) / (double)((*task_it)->p_);
			
			simulator_core_task_mappings_[min_util_core].insert((*task_it)->name_);
			simulator_task_core_mappings_[(*task_it)->name_] = min_util_core;

			std::cerr << (*task_it)->name_ << "::" << min_util_core << "::" << min_util << std::endl;
		}
	}
	//worst-fit first 
	else if (heuristics == AbstractedControlSystem::heuristics_wff)
	{
		//사용하지 않은 용량 
		std::vector<std::uint64_t> simulator_mem_bin(simulator_num_of_cores_, 256);
		
		for (auto task_it = tasks_.begin(); task_it != tasks_.end(); ++task_it)
		{
			
			std::uint64_t max_mem = std::numeric_limits<uint64_t>::min();
			std::uint64_t max_mem_core = 0;

			for (std::uint64_t core = 0; core < simulator_num_of_cores_; ++core)
			{
				if (simulator_mem_bin[core] >= (*task_it)->memory_usage_ && max_mem < simulator_mem_bin[core])
				{
					//남은 용량이 최대인 core와 그 메모리 양을 찾음 
					max_mem = simulator_mem_bin[core]; 
					max_mem_core = core;
				}
			}
			
			if (max_mem == std::numeric_limits<std::uint64_t>::min())
			{
				std::cout << "Not Simulatable (E: " << ecus_.size() << ", T:" << tasks_.size() << ")::memory_constraint_violation" << std::endl;
				exit(-1);
			}

			simulator_mem_bin[max_mem_core] -= (*task_it)->memory_usage_;
			
			simulator_core_task_mappings_[max_mem_core].insert((*task_it)->name_);
			simulator_task_core_mappings_[(*task_it)->name_] = max_mem_core;
			
			std::cerr << (*task_it)->name_ << "::" << max_mem_core << "::" << max_mem << std::endl;
		}
	}
	else if (heuristics == AbstractedControlSystem::heuristics_sbf)
	{
		//memory양 (256kb)
		std::vector<std::uint64_t> simulator_mem_bin(simulator_num_of_cores_, 256);
		//smallest block 찾으려고 각 core마다 block값을 저장 
		std::vector<std::vector<std::string> > simulator_block_bins;

		AbstractedJob* vs_sparse = initialize_sparse_graph();
		assign_epst(vs_sparse);

		AbstractedJob* vs_dense = initialize_dense_graph();
		assign_lpft(vs_dense);

		simulator_block_bins.assign(simulator_num_of_cores_, std::vector<std::string>());

		for (auto task_to_be_added = tasks_.begin(); task_to_be_added != tasks_.end(); ++task_to_be_added)
		{
			std::uint64_t min_block_core;
			double min_block_value = std::numeric_limits<double>::max();
			
			for (std::uint64_t core = 0; core < simulator_num_of_cores_; ++core)
			{
				//남은 양이 현재 task의 memory usage보다 작다면 못 넣기 때문에 pass 
				if (simulator_mem_bin[core] < (*task_to_be_added)->memory_usage_)
					continue;
				
				double block_value = 0;

				for (auto job_it = (*task_to_be_added)->pended_ojpg_jobs_.begin(); job_it != (*task_to_be_added)->pended_ojpg_jobs_.end(); ++job_it)
				{
					//if the job is not hat job, then continue
					if ((*job_it)->name_.find("^") != std::string::npos)
						continue;

					std::uint64_t my_epst = (*get_sparse_graph_job_by_name((*job_it)->name_))->epst_;
					std::uint64_t my_lpft = (*get_dense_graph_job_by_name((*job_it)->name_))->lpft_;
					double my_weight = ((double)((*task_to_be_added)->c_worst_) / (double)simulator_performance_ratio_) / ((double)my_lpft - (double)my_epst);
					
					for (auto existing_job = simulator_block_bins[core].begin(); existing_job != simulator_block_bins[core].end(); ++existing_job)
					{
						if (existing_job->find("^") != std::string::npos)
							continue;

						AbstractedJob* your_job = (*get_ojpg_job_by_name(*existing_job));

						std::uint64_t your_epst = (*get_sparse_graph_job_by_name(*existing_job))->epst_;
						std::uint64_t your_lpft = (*get_dense_graph_job_by_name(*existing_job))->lpft_;
						double your_weight = ((double)(your_job->mapped_task_->c_worst_) / (double)simulator_performance_ratio_ / ((double)your_lpft - (double)your_epst));

						if (my_epst <= your_epst && my_lpft > your_epst)
							block_value += (double)(std::min<std::uint64_t>(my_lpft, your_lpft) - your_epst) * my_weight * your_weight;
						else if (my_epst > your_epst && my_epst < your_lpft)
							block_value += (double)(std::min<std::uint64_t>(my_lpft, your_lpft) - my_epst) * my_weight * your_weight;
					}
				}
				
				if (block_value < min_block_value)
				{
					min_block_value = block_value;
					min_block_core = core;
				}

				if (min_block_value == std::numeric_limits<double>::max() )
				{
					std::cout << " Not Simulatable (E: " << ecus_.size() << ", T" << tasks_.size() << ")::memory_constraint_violation" << std::endl;
					exit(-1);
				}
			}
			simulator_mem_bin[min_block_core] -= (*task_to_be_added)->memory_usage_;
			
			simulator_core_task_mappings_[min_block_core].insert((*task_to_be_added)->name_);
			simulator_task_core_mappings_[(*task_to_be_added)->name_] = min_block_core;

			std::cerr << (*task_to_be_added)->name_ << "::" << min_block_core << "::" << min_block_value << std::endl;

			for (auto job_it = (*task_to_be_added)->pended_ojpg_jobs_.begin(); job_it != (*task_to_be_added)->pended_ojpg_jobs_.end(); ++job_it)
				simulator_block_bins[min_block_core].push_back((*job_it)->name_);
		}
		
		delete vs_sparse;
		delete vs_dense;
	}
	





}

/**
  * @brief	Initialize OJPG from offline guider
	*					Deep copy 1st HP's jobs of offline guider to the initial OJPG
	*					Assign initial deadlines to the offline guider
	*/
void AbstractedControlSystem::initialize_ojpg(void)
{
	//Deep copy the second-HP's jobs
	for (auto offline_guider_it = offline_guider_.begin(); offline_guider_it != offline_guider_.end(); ++offline_guider_it)
	{
		//means its jobs is in 2nd-HP
		if ((*offline_guider_it)->which_period_ == 1)
		{
			//copy-constructor-> *offline_guider_it : AbstractedJob* 형태, const AbstractedJob& ref로 주려면 AbstractedJob object 필요 따라서 **offline_guider_it이 AbstractedJob을 pointing
			AbstractedJob* ojpg_job = new AbstractedJob(*(*offline_guider_it));
			ojpg_job->mapped_task_->pended_ojpg_jobs_.push_back(ojpg_job);
			ojpg_.push_back(ojpg_job);
		}
	}

	//Replace inks of copied jobs and adjust its timing parameter
	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
	{
		AbstractedJob* offline_guider_job = *get_offline_guider_job_by_name((*ojpg_it)->name_);
		
		auto replacer = [&](std::set<AbstractedJob*>& ojpg_link, std::set<AbstractedJob*>& offline_guider_link)
		{
			ojpg_link.clear();
			for (auto offline_guider_it = offline_guider_link.begin(); offline_guider_it != offline_guider_link.end(); ++offline_guider_it)
			{
				//get ojpg_job_by_name return type iterator -> *달아서 AbstractedJob* type으로 changing
				if ((*offline_guider_it)->which_period_ == 1)
					ojpg_link.insert(*get_ojpg_job_by_name( (*offline_guider_it)->name_) );
			}


		};
		
		replacer((*ojpg_it)->j_prev_det_predecessors_, offline_guider_job->j_prev_det_predecessors_);
		replacer((*ojpg_it)->j_prev_det_successors_, offline_guider_job->j_prev_det_successors_);

		replacer((*ojpg_it)->j_s_, offline_guider_job->j_s_);
		replacer((*ojpg_it)->j_s_det_predecessors_, offline_guider_job->j_s_det_predecessors_);
		replacer((*ojpg_it)->j_s_det_successors_, offline_guider_job->j_s_det_successors_);
		replacer((*ojpg_it)->j_p_nodet_predecessors_, offline_guider_job->j_p_nodet_predecessors_);
		replacer((*ojpg_it)->j_p_nodet_successors_, offline_guider_job->j_p_nodet_successors_);

		replacer((*ojpg_it)->j_f_, offline_guider_job->j_f_);
		replacer((*ojpg_it)->j_f_det_predecessors_, offline_guider_job->j_f_det_predecessors_);
		replacer((*ojpg_it)->j_f_det_successors_, offline_guider_job->j_f_det_successors_);
		replacer((*ojpg_it)->j_f_nodet_predecessors_, offline_guider_job->j_f_nodet_predecessors_);
		replacer((*ojpg_it)->j_f_nodet_successors_, offline_guider_job->j_f_nodet_successors_);

		replacer((*ojpg_it)->j_p_, offline_guider_job->j_p_);
		replacer((*ojpg_it)->j_p_det_predecessors_, offline_guider_job->j_p_det_predecessors_);
		replacer((*ojpg_it)->j_p_det_successors_, offline_guider_job->j_p_det_successors_);
		replacer((*ojpg_it)->j_p_nodet_predecessors_, offline_guider_job->j_p_nodet_predecessors_);
		replacer((*ojpg_it)->j_p_nodet_successors_, offline_guider_job->j_p_nodet_successors_);

		(*ojpg_it)->t_r_real_-=hyper_period_;
		(*ojpg_it)->min_t_s_real_ -= hyper_period_;
		(*ojpg_it)->max_t_s_real_ -= hyper_period_;
		(*ojpg_it)->min_t_f_real_ -= hyper_period_;
		(*ojpg_it)->max_t_f_real_ -= hyper_period_;
	}
	assign_ojpg_deadlines();
}

/**
  * @brief	Assign deadline for each job included in OJPG
	*					This assignment is iterated until convergence occurs
	*
	*/
void AbstractedControlSystem::assign_ojpg_deadlines(void)
{
	// Assign deadline : initial value
	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ojpg_it)
	{
		if ((*ojpg_it)->name_.find("^") != std::string::npos)
		{
			//assign deadline for each minimum finish time 
			std::string non_hat_job_name = std::string((*ojpg_it)->name_.begin(), (*ojpg_it)->name_.end() - 1);
			(*ojpg_it)->t_d_sim_ = (*get_ojpg_job_by_name(non_hat_job_name))->min_t_f_real_;
		}
		//assign deadline to maximum value of uint64_t
		else
			(*ojpg_it)->t_d_sim_ = std::numeric_limits<std::uint64_t>::max();
	}

	//Assign deadline : iterate for convergence 
	while (true)
	{
		std::uint32_t number_of_updated_jobs = 0;
		
		for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ojpg_it++)
		{
			std::uint64_t new_t_d_sim = std::numeric_limits<std::uint64_t>::max();

			//minimum 값만 찾기 때문에 constraint 다 돌면서 제일 작은 값

			//Previous release relation successors
			for (auto succ_it = (*ojpg_it)->j_prev_det_successors_.begin(); succ_it != (*ojpg_it)->j_prev_det_predecessors_.end(); ++succ_it)
				new_t_d_sim = std::min<uint64_t>(new_t_d_sim, (*succ_it)->t_d_sim_);

			//Start-time set relation successorss
			for (auto succ_it = (*ojpg_it)->j_s_det_successors_.begin(); succ_it != (*ojpg_it)->j_s_det_successors_.end(); ++succ_it)
				new_t_d_sim = std::min<uint64_t>(new_t_d_sim, (*succ_it)->t_d_sim_);

			//Finish-time set relation successors
			for (auto succ_it = (*ojpg_it)->j_f_det_successors_.begin(); succ_it != (*ojpg_it)->j_f_det_successors_.end(); ++succ_it)
				new_t_d_sim = std::min<uint64_t>(new_t_d_sim, (*succ_it)->t_d_sim_);

			//Producer-time set relation successors
			for (auto succ_it = (*ojpg_it)->j_p_det_successors_.begin(); succ_it != (*ojpg_it)->j_p_det_successors_.end(); ++succ_it)
				new_t_d_sim = std::min<uint64_t>(new_t_d_sim, (*succ_it)->t_d_sim_);

			if (new_t_d_sim < (*ojpg_it)->t_d_sim_)
			{
				(*ojpg_it)->t_d_sim_ = new_t_d_sim;
				++number_of_updated_jobs;
			}
		}

		if (number_of_updated_jobs == 0)
			break;
	}
}



/**
  * @brief	Update OJPG
	*					- Adding next HP's jobs of completed jobs
	*				  - Narrowing execution windows using completed jobs' virtually assigned execution time
	*					- Deleting completed jobs from OJPG
	*					- Resolving nondeterminism if it is possible
	*					- Re-assigning deadline for each job in OJPG
	*
	*
	*/
void AbstractedControlSystem::update_ojpg(
	std::vector<AbstractedJob*>& completed_jobs)
{
	update_ojpg_add_new_jobs(completed_jobs);

	update_ojpg_narrow_execution_windows();

	update_ojpg_delete_completed_jobs(completed_jobs);

	update_ojpg_resolve_nondeterminism();

	assign_ojpg_deadlines();
}

/** 
  * @brief	Add a newly created jobs due to job completion to the OJPG
	*					Newly created jobs are next HP's jobs of completed jobs
	*
	* @param	completed_jobs	jobs which is completed on simulator
	*/
void AbstractedControlSystem::update_ojpg_add_new_jobs(
	std::vector<AbstractedJob*>& completed_jobs)
{
	for (auto completed_jobs_it = completed_jobs.begin(); completed_jobs_it != completed_jobs.end(); ++completed_jobs_it)
	{
		
		// Copy and add a node
		AbstractedTask *job_to_be_added_mapped_task = (*completed_jobs_it)->mapped_task_;
		
		std::uint64_t job_to_be_added_job_idx;
		//끝난 job의 index 먼저 구하고 
		job_to_be_added_job_idx = AbstractedJob::get_job_index((*completed_jobs_it)->name_);
		//hyperPeriod/Period 해서 다음 hyper_period까지의 job의 index를 구한다 -> 만약에 hyperPeriod에서 P 나눈 값이 1이 아닌 경우?, 즉 2 이상인 경우 어캄? 
		//->한 tick마다 진행하므로 무조건 하나 존재, J_11 J_12 가 한 HP라 하면, J_11 수행 후 J_13 생성 
		job_to_be_added_job_idx += (hyper_period_ / job_to_be_added_mapped_task->p_);

		std::string job_to_be_added_job_name;
		job_to_be_added_job_name = AbstractedJob::assemble_job_name((*completed_jobs_it)->name_, job_to_be_added_job_idx);
		
		std::uint64_t job_to_be_added_which_period;
		job_to_be_added_which_period = AbstractedJob::get_job_index(job_to_be_added_job_name) / (hyper_period_ / job_to_be_added_job_idx);

		AbstractedJob *job_to_be_added = new AbstractedJob(job_to_be_added_job_name, job_to_be_added_mapped_task, job_to_be_added_which_period);
		job_to_be_added->t_r_real_ = (*completed_jobs_it)->t_r_real_ + hyper_period_;
		job_to_be_added->mapped_task_->pended_ojpg_jobs_.push_back(job_to_be_added);
		ojpg_.push_back(job_to_be_added);

		//Find corresponding jobs on offline guider
		std::uint64_t corresponding_offline_guider_job_in_0th_hp_idx;
		std::uint64_t corresponding_offline_guider_job_in_1st_hp_idx;

		corresponding_offline_guider_job_in_0th_hp_idx = AbstractedJob::get_job_index(job_to_be_added_job_name);
		corresponding_offline_guider_job_in_0th_hp_idx %= (hyper_period_ / job_to_be_added_mapped_task->p_);

		corresponding_offline_guider_job_in_1st_hp_idx = AbstractedJob::get_job_index(job_to_be_added_job_name);
		corresponding_offline_guider_job_in_1st_hp_idx %= (hyper_period_ / job_to_be_added_mapped_task->p_);
		corresponding_offline_guider_job_in_1st_hp_idx += (hyper_period_ / job_to_be_added_mapped_task->p_);
		
		std::string corresponding_offline_guider_job_in_0th_hp_name;
		std::string corresponding_offline_guider_job_in_1st_hp_name;

		corresponding_offline_guider_job_in_0th_hp_name = AbstractedJob::assemble_job_name(job_to_be_added_job_name, corresponding_offline_guider_job_in_0th_hp_idx);
		corresponding_offline_guider_job_in_1st_hp_name = AbstractedJob::assemble_job_name(job_to_be_added_job_name, corresponding_offline_guider_job_in_1st_hp_idx);

		AbstractedJob* corresponding_offline_guider_job_in_0th_hp = *get_offline_guider_job_by_name(corresponding_offline_guider_job_in_0th_hp_name);
		AbstractedJob* corresponding_offline_guider_job_in_1st_hp = *get_offline_guider_job_by_name(corresponding_offline_guider_job_in_1st_hp_name);

		update_ojpg_copy_links(job_to_be_added, corresponding_offline_guider_job_in_0th_hp);
		update_ojpg_copy_links(job_to_be_added, corresponding_offline_guider_job_in_1st_hp);
	}
}

/**
  * @brief	Copy links of offline guider job to the newly added ojpg job
	*
	*
	* @param	ojpg_job													job which is newly added to the ojpg
	* @param	corresponding_offline_guider_job	corresponding offline guider job of ojpg_job
	*/
void AbstractedControlSystem::update_ojpg_copy_links(
	AbstractedJob* ojpg_job,
	AbstractedJob* corresponding_offline_guider_job)
{
	std::set<AbstractedJob*>::iterator link_begin, link_end;
	for (std::uint16_t links_type = 0; links_type < 14; ++links_type)
	{
		switch (links_type)
		{
		case 0:
			link_begin = corresponding_offline_guider_job->j_prev_det_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_prev_det_predecessors_.end();
			break;
		case 1:
			link_begin = corresponding_offline_guider_job->j_prev_det_successors_.begin();
			link_end = corresponding_offline_guider_job->j_prev_det_successors_.end();
			break;
		case 2:
			link_begin = corresponding_offline_guider_job->j_s_det_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_s_det_predecessors_.end();
			break;
		case 3:
			link_begin = corresponding_offline_guider_job->j_s_det_successors_.begin();
			link_end = corresponding_offline_guider_job->j_s_det_successors_.end();
			break;
		case 4:
			link_begin = corresponding_offline_guider_job->j_s_nodet_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_s_nodet_predecessors_.end();
			break;
		case 5:
			link_begin = corresponding_offline_guider_job->j_s_nodet_successors_.begin();
			link_end = corresponding_offline_guider_job->j_s_nodet_successors_.end();
			break;
		case 6:
			link_begin = corresponding_offline_guider_job->j_f_det_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_f_det_predecessors_.end();
			break;
		case 7:
			link_begin = corresponding_offline_guider_job->j_f_det_successors_.begin();
			link_end = corresponding_offline_guider_job->j_f_det_successors_.end();
		case 8:
			link_begin = corresponding_offline_guider_job->j_f_nodet_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_f_nodet_predecessors_.end();
			break;
		case 9:
			link_begin = corresponding_offline_guider_job->j_f_nodet_successors_.begin();
			link_end = corresponding_offline_guider_job->j_f_nodet_successors_.end();
			break;
		case 10:
			link_begin = corresponding_offline_guider_job->j_p_det_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_p_det_predecessors_.begin();
			break;
		case 11:
			link_begin = corresponding_offline_guider_job->j_p_det_successors_.begin();
			link_end = corresponding_offline_guider_job->j_p_det_successors_.end();
			break;
		case 12:
			link_begin = corresponding_offline_guider_job->j_p_nodet_predecessors_.begin();
			link_end = corresponding_offline_guider_job->j_p_nodet_predecessors_.end();
			break;
		case 13:
			link_begin = corresponding_offline_guider_job->j_p_nodet_successors_.begin();
			link_end = corresponding_offline_guider_job->j_p_nodet_successors_.end();
			break;
		}

		std::uint64_t ojpg_job_which_period = ojpg_job->which_period_;
		std::uint64_t corresponding_offline_guider_job_which_period = corresponding_offline_guider_job->which_period_;

		std::uint64_t period_diff = ojpg_job_which_period - corresponding_offline_guider_job_which_period;

		for (auto link_it = link_begin; link_it != link_end; ++link_it)
		{
			AbstractedJob *linked_job = *link_it;
			
			//edge걸려있는 놈들의 idx 값 빼내오기 
			std::uint64_t expected_ojpg_job_idx;
			expected_ojpg_job_idx = AbstractedJob::get_job_index(linked_job->name_);
			expected_ojpg_job_idx += (period_diff) * (hyper_period_ / linked_job->mapped_task_->p_);

			std::string expected_ojpg_job_name;
			expected_ojpg_job_name = AbstractedJob::assemble_job_name(linked_job->name_, expected_ojpg_job_idx);

			if (get_ojpg_job_by_name(expected_ojpg_job_name) == ojpg_.end())
				continue;

			//offline-guider에서 relation예상되는 job 
			AbstractedJob *expected_ojpg_job = *get_ojpg_job_by_name(expected_ojpg_job_name);
			switch (links_type)
			{
			case 0:
				ojpg_job->j_prev_det_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_prev_det_successors_.insert(ojpg_job);
				break;
			case 1:
				ojpg_job->j_prev_det_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_prev_det_predecessors_.insert(ojpg_job);
				break;
			case 2:
				ojpg_job->j_s_det_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_s_det_successors_.insert(ojpg_job);
				break;
			case 3:
				ojpg_job->j_s_det_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_s_det_predecessors_.insert(ojpg_job);
				break;
			case 4:
				ojpg_job->j_s_nodet_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_s_nodet_successors_.insert(ojpg_job);
				break;
			case 5:
				ojpg_job->j_s_nodet_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_s_nodet_predecessors_.insert(ojpg_job);
				break;
			case 6:
				ojpg_job->j_f_det_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_f_det_successors_.insert(ojpg_job);
				break;
			case 7:
				ojpg_job->j_f_det_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_f_det_predecessors_.insert(ojpg_job);
				break;
			case 8:
				ojpg_job->j_f_nodet_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_f_nodet_successors_.insert(ojpg_job);
				break;
			case 9:
				ojpg_job->j_f_nodet_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_f_nodet_predecessors_.insert(ojpg_job);
				break;
			case 10:
				ojpg_job->j_p_det_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_p_det_successors_.insert(ojpg_job);
				break;
			case 11:
				ojpg_job->j_p_det_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_p_det_predecessors_.insert(ojpg_job);
				break;
			case 12:
				ojpg_job->j_p_nodet_predecessors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_p_nodet_successors_.insert(ojpg_job);
				break;
			case 13:
				ojpg_job->j_p_det_successors_.insert(expected_ojpg_job);
				(expected_ojpg_job)->j_p_det_predecessors_.insert(ojpg_job);
				break;
			}
		}
	}
}

/*
 * @brief	Narrow start/finish-time range of OJPG jobs using completed jobs
 *
 */
void AbstractedControlSystem::update_ojpg_narrow_execution_windows(void)
{
	//1 copy다 뜨고 나서 
	//2 execution window 좁히고
	//3 completed job delete
	//4 nondeterminism resolve
	//5 deadline assign 

	//we are in step 2 for this function 
	//mode : 0 -> best execution time window narrowing 
	//mode : 1 -> worst execution time window narrowing

	for (auto ecu_it = ecus_.begin(); ecu_it != ecus_.end(); ++ecu_it)
	{
		auto filler = [&](std::map<std::string, time_window_t>& window, const std::uint64_t num_of_hyper_periods, const std::uint8_t mode)
		{
			window[(*ecu_it)->name_] = time_window_t(num_of_hyper_periods * hyper_period_, "EMPTY");

			for (auto task_it = (*ecu_it)->pended_tasks_.begin(); task_it != (*ecu_it)->pended_tasks_.end(); ++task_it)
			{
				//vector constructor pended job의 execution window를 줄이는 것 
				std::vector<AbstractedJob*> jobs_to_be_updated((*task_it)->pended_ojpg_jobs_.begin(), (*task_it)->pended_ojpg_jobs_.end());
				//position에서 pneded_job먼저 넣고 pended_completed_ojpg_job을 insert 
				jobs_to_be_updated.insert(jobs_to_be_updated.end(), (*task_it)->pended_completed_ojpg_jobs_.begin(), (*task_it)->pended_completed_ojpg_jobs_.end());

				for (auto job_it = jobs_to_be_updated.begin(); job_it != jobs_to_be_updated.end(); ++job_it)
				{
					//hat-job then, no more stuffs to do 
					if ((*job_it)->name_.find("^") != std::string::npos)
						continue;

					std::uint64_t executed_time = 0;
					std::uint64_t current_time = (*job_it)->t_r_real_;

					std::uint64_t job_execution_time;
					if ((*job_it)->executed_time_ == (*job_it)->e_sim_)
						job_execution_time = (*job_it)->e_real_;
					else if (mode == 0)
						job_execution_time = (*task_it)->c_best_;
					else if (mode == 1)
						job_execution_time = (*task_it)->c_worst_;
						
					while (executed_time < job_execution_time)
					{
						if (window[(*ecu_it)->name_][current_time].compare("EMPTY") == 0)
						{
							if (executed_time == 0)
								(mode == 0 ? (*job_it)->min_t_s_real_ : (*job_it)->max_t_s_real_) = current_time;

							window[(*ecu_it)->name_][current_time] = (*job_it)->name_;
							++executed_time;
						}
						++current_time;
					}
					(mode == 0 ? (*job_it)->min_t_f_real_ : (*job_it)->max_t_f_real_) = current_time;
				}
			}
		};
		
		std::uint64_t max_hyper_periods = 1;
		for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
		{
			AbstractedTask *mapped_task = (*ojpg_it)->mapped_task_;
			std::uint64_t which_period = AbstractedJob::get_job_index((*ojpg_it)->name_) / (hyper_period_ / mapped_task->p_);
			
			max_hyper_periods = std::max<uint64_t>(max_hyper_periods, which_period);
		}

		filler(best_case_execution_window_, max_hyper_periods, 0);
		filler(worst_case_execution_window_, max_hyper_periods, 1);
	}

	
	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
	{
		if ((*ojpg_it)->name_.find("^") == std::string::npos)
			continue;

		std::string non_hat_job_name = std::string((*ojpg_it)->name_.begin(), (*ojpg_it)->name_.end() - 1);
		
		if (get_ojpg_job_by_name(non_hat_job_name) != ojpg_.end())
		{
			AbstractedJob* non_hat_job = *get_ojpg_job_by_name(non_hat_job_name);
			(*ojpg_it)->min_t_s_real_ = non_hat_job->min_t_f_real_;
			(*ojpg_it)->max_t_s_real_ = non_hat_job->max_t_f_real_;
			(*ojpg_it)->min_t_f_real_ = (*ojpg_it)->min_t_s_real_;
			(*ojpg_it)->max_t_f_real_ = (*ojpg_it)->max_t_s_real_;
		}
	}
}

/**
  * @ brief	Delete completed jobs from OJPG
	*
	*
	* @param	completed_jobs	jobs which is completed on simulator
	*/
void AbstractedControlSystem::update_ojpg_delete_completed_jobs(
	std::vector<AbstractedJob*>& completed_jobs)
{
	for (auto completed_job_it = completed_jobs.begin(); completed_job_it != completed_jobs.end(); ++completed_job_it)
	{
		auto delete_expired_link = [&](std::set<AbstractedJob*>& links, std::uint16_t mode)
		{
			for (auto link_it = links.begin(); link_it != links.end(); ++link_it)
			{
				switch (mode)
				{
				case 0:
					(*link_it)->j_prev_det_predecessors_.erase(*completed_job_it);
					break;
				case 1:
					(*link_it)->j_s_det_predecessors_.erase(*completed_job_it);
					break;
				case 2:
					(*link_it)->j_s_nodet_predecessors_.erase(*completed_job_it);
					break;
				case 3:
					(*link_it)->j_f_det_predecessors_.erase(*completed_job_it);
					break;
				case 4:
					(*link_it)->j_f_nodet_predecessors_.erase(*completed_job_it);
					break;
				case 5:
					(*link_it)->j_f_det_predecessors_.erase(*completed_job_it);
					break;
				case 6:
					(*link_it)->j_f_nodet_predecessors_.erase(*completed_job_it);
					break;
				}
			}
		};
		
		delete_expired_link((*completed_job_it)->j_prev_det_predecessors_, 0);
		delete_expired_link((*completed_job_it)->j_s_det_predecessors_, 1);
		delete_expired_link((*completed_job_it)->j_s_nodet_predecessors_, 2);
		delete_expired_link((*completed_job_it)->j_f_det_predecessors_, 3);
		delete_expired_link((*completed_job_it)->j_f_nodet_predecessors_, 4);
		delete_expired_link((*completed_job_it)->j_p_det_predecessors_, 5);
		delete_expired_link((*completed_job_it)->j_p_nodet_predecessors_, 6);

		AbstractedTask *mapped_task = (*completed_job_it)->mapped_task_;

		std::vector<AbstractedJob*>::iterator del_pos;
		
		del_pos = std::find(mapped_task->pended_ojpg_jobs_.begin(), mapped_task->pended_ojpg_jobs_.end(), (*completed_job_it));
		mapped_task->pended_completed_ojpg_jobs_.erase(del_pos);

		del_pos = std::find(ojpg_.begin(), ojpg_.end(), (*completed_job_it));
		ojpg_.erase(del_pos);
		
		mapped_task->pended_completed_ojpg_jobs_.push_back(*completed_job_it);
		completed_ojpg_.push_back(*completed_job_it);

	}
}

/**
  * @brief	Resolve nondeterminism using narrowed start/finish-time range
	*
	*/
void AbstractedControlSystem::update_ojpg_resolve_nondeterminism(void)
{
	for (auto ojpg_it = ojpg_.begin(); ojpg_it != ojpg_.end(); ++ojpg_it)
	{
		//Resolve nondeterminism generated by physical read constraint(related to start time set)
		for (auto succ_it = (*ojpg_it)->j_s_nodet_predecessors_.begin(); succ_it != (*ojpg_it)->j_s_nodet_successors_.end(); )
		{
			AbstractedJob* j_kl = (*ojpg_it);
			AbstractedJob* j_ij = (*succ_it);
			
			//det-edge 되거나 
			if (j_kl->max_t_s_real_ < j_ij->min_t_s_real_)
			{
				succ_it = j_kl->j_s_nodet_successors_.erase(succ_it);
				j_ij->j_s_nodet_predecessors_.erase(j_kl);

				j_kl->j_s_det_successors_.insert(j_ij);
				j_ij->j_s_det_predecessors_.insert(j_kl);
			}
			//없어지거나 
			else if (j_kl->min_t_s_real_ >= j_ij->max_t_s_real_)
			{
				succ_it = j_kl->j_s_nodet_successors_.erase(succ_it);
				j_ij->j_s_nodet_predecessors_.erase(j_kl);

			}
			else
				++succ_it;
		}

		// Resolve nondeterminism generated by physical write constraint (related to finish-time set)
		for (auto succ_it = (*ojpg_it)->j_f_nodet_successors_.begin(); succ_it != (*ojpg_it)->j_f_nodet_successors_.end();)
		{
			AbstractedJob* j_kl = (*ojpg_it);
			AbstractedJob* j_ij = (*succ_it);

			if (j_ij->name_.find("^") != std::string::npos)
			{
				++succ_it;
				continue;
			}

			if (j_kl->max_t_s_real_ < j_ij->min_t_f_real_)
			{
				succ_it = j_kl->j_f_nodet_successors_.erase(succ_it);
				j_ij->j_f_nodet_predecessors_.erase(j_kl);

				j_kl->j_f_det_successors_.insert(j_ij);
				j_ij->j_f_det_predecessors_.insert(j_kl);
			}
			else if (j_kl->min_t_s_real_ >= j_ij->max_t_f_real_)
			{
				succ_it = j_kl->j_f_nodet_successors_.erase(succ_it);
				j_ij->j_f_nodet_predecessors_.erase(j_kl);
			}
			else
				++succ_it;
		}

		//Resolve nondeterminsm generated by producer consumer constraint
		for (auto succ_it = (*ojpg_it)->j_p_nodet_successors_.begin(); succ_it != (*ojpg_it)->j_p_nodet_successors_.end(); )
		{
			AbstractedJob *j_kl = (*ojpg_it);
			AbstractedJob *j_ij = (*succ_it);

			if (j_kl->max_t_s_real_ < j_ij->min_t_s_real_)
			{
				succ_it = j_kl->j_p_nodet_successors_.erase(succ_it);
				j_ij->j_p_nodet_predecessors_.erase(j_kl);

				j_kl->j_p_det_successors_.insert(j_ij);
				j_ij->j_p_det_predecessors_.insert(j_kl);
			}
			else if (j_kl->min_t_s_real_ >= j_ij->max_t_s_real_)
			{
				succ_it = j_kl->j_p_nodet_successors_.erase(succ_it);
				j_ij->j_p_nodet_predecessors_.erase(j_kl);
			}
			else
				++succ_it;
		}
	}
}

std::vector<AbstractedEcu*>::iterator AbstractedControlSystem::get_ecu_by_name(
	const std::string ecu_name)
{
	std::vector<AbstractedEcu*>::iterator ret;
	for (ret = ecus_.begin(); ret != ecus_.end(); ++ret)
	{
		if ((*ret)->name_.compare(ecu_name) == 0)
			break;
	}
	return ret;
}

std::vector<AbstractedTask*>::iterator AbstractedControlSystem::get_task_by_name(
	const std::string task_name)
{
	std::vector<AbstractedTask*>::iterator ret;
	for (ret = tasks_.begin(); ret != tasks_.end(); ++ret)
	{
		if ((*ret)->name_.compare(task_name) == 0)
			break;
	}
	return ret;
}

std::vector<AbstractedJob*>::iterator AbstractedControlSystem::get_offline_guider_job_by_name(
	const std::string job_name)
{
	std::vector<AbstractedJob*>::iterator ret;
	for (ret = offline_guider_.begin(); ret != offline_guider_.end(); ++ret)
	{
		if ((*ret)->name_.compare(job_name) == 0)
			break;
	}
	return ret;
}

std::vector<AbstractedJob*>::iterator AbstractedControlSystem::get_ojpg_job_by_name(
	const std::string job_name)
{
	std::vector<AbstractedJob*>::iterator ret;
	for (ret = ojpg_.begin(); ret != ojpg_.end(); ++ret)
	{
		if ((*ret)->name_.compare(job_name) == 0)
			break;
	}
	return ret;
}

std::vector<AbstractedJob*>::iterator AbstractedControlSystem::get_sparse_graph_job_by_name(
	const std::string job_name)
{
	std::vector<AbstractedJob*>::iterator ret;
	for (ret = sparse_graph_.begin(); ret != sparse_graph_.end(); ++ret)
	{
		if ((*ret)->name_.compare(job_name) == 0)
			break;
	}
	return ret;
}

std::vector<AbstractedJob*>::iterator AbstractedControlSystem::get_dense_graph_job_by_name(
	const std::string job_name)
{
	std::vector<AbstractedJob*>::iterator ret;
	for (ret = dense_graph_.begin(); ret != dense_graph_.end(); ++ret)
	{
		if ((*ret)->name_.compare(job_name) == 0)
			break;
	}
	return ret;
}

template<class T>
T AbstractedControlSystem::get_gcd(
	const T a,
	const T b)
{
	if (b == 0)
		return a;
	else
		return get_gcd(b, a%b);
}

template<class T>
T AbstractedControlSystem::get_lcm(
	const std::vector<T>& number_array)
{
	if (number_array.empty())
		return T(0);
	
	std::set<T> unique_number_array;
	for (auto it = number_array.begin(); it != number_array.end(); ++it)
		unique_number_array.insert((*it));
	
	T lcm = *unique_number_array.begin();
	auto it = unique_number_array.begin();

	++it;
	for (; it != unique_number_array.end(); ++it)
		lcm = ((*it)*lcm) / (AbstractedControlSystem::get_gcd<std::uint64_t>(*it, lcm));

	return lcm;
}

/**
  * @brief	Topological sort of graph rooted at root
	*
	*
	* @param	root			root of DFS forest
	* @param	stack			stack for tracing
	* @param	visited		visited flags
	*/
void AbstractedControlSystem::topological_sort(
	AbstractedJob* root,
	std::vector<AbstractedJob*>& stack,
	std::map<AbstractedJob*, bool>& visited)
{
	visited[root] = true;
	
	for (auto edge_it = root->edge_weights_.begin(); edge_it != root->edge_weights_.end(); ++edge_it)
	{
		if(visited[edge_it->first]==false)
			topological_sort(edge_it->first, stack, visited);
	}
	
	stack.push_back(root);
	
	return;
}

/**
  * @brief	Initialize sparse graph which does not have ND-edges
	*
	*
	* @return	Virtual start job
	*/
AbstractedJob* AbstractedControlSystem::initialize_sparse_graph(void)
{
	//Deep copy the second-HP's jobs
	for (auto offline_guider_it = offline_guider_.begin(); offline_guider_it != offline_guider_.end(); ++offline_guider_it)
	{
		if ((*offline_guider_it)->which_period_ == 1) // only copy second period jobs
		{
			AbstractedJob* sparse_graph_job = new AbstractedJob(*(*offline_guider_it));
			sparse_graph_job->mapped_task_->pended_sparse_graph_jobs_.push_back(sparse_graph_job);
			sparse_graph_.push_back(sparse_graph_job);
		}
	}

	//Replace links of copied jobs and adjust its timing parameter
	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); ++sparse_graph_it)
	{
		AbstractedJob* offline_guider_job = *get_offline_guider_job_by_name((*sparse_graph_it)->name_);

		auto replacer = [&](std::set<AbstractedJob*>& sparse_graph_link, std::set<AbstractedJob*>& offline_guider_link)
		{
			sparse_graph_link.clear();
			for (auto offline_guider_it = offline_guider_link.begin(); offline_guider_it != offline_guider_link.end(); ++offline_guider_it)
			{
				if ((*offline_guider_it)->which_period_ == 1) // Only the links among the 2nd HP should be copied
					sparse_graph_link.insert(*get_sparse_graph_job_by_name((*offline_guider_it)->name_));
			}
		};

		//Copy only deterministic edges 

		replacer((*sparse_graph_it)->j_prev_det_predecessors_, offline_guider_job->j_prev_det_predecessors_);
		replacer((*sparse_graph_it)->j_prev_det_successors_, offline_guider_job->j_prev_det_successors_);

		replacer((*sparse_graph_it)->j_s_det_predecessors_, offline_guider_job->j_s_det_predecessors_);
		replacer((*sparse_graph_it)->j_s_det_successors_, offline_guider_job->j_s_det_successors_);

		replacer((*sparse_graph_it)->j_f_det_predecessors_, offline_guider_job->j_f_det_predecessors_);
		replacer((*sparse_graph_it)->j_f_det_successors_, offline_guider_job->j_f_det_successors_);

		replacer((*sparse_graph_it)->j_p_det_predecessors_, offline_guider_job->j_p_det_predecessors_);
		replacer((*sparse_graph_it)->j_p_det_successors_, offline_guider_job->j_p_det_successors_);

		//2nd HP 이므로 1st(from the view of sparse graph)로 만들어주려고 hp값 만큼 빼준다 
		(*sparse_graph_it)->t_r_real_ -= hyper_period_;
		(*sparse_graph_it)->min_t_s_real_ -= hyper_period_;
		(*sparse_graph_it)->max_t_s_real_ -= hyper_period_;
		(*sparse_graph_it)->min_t_f_real_ -= hyper_period_;
		(*sparse_graph_it)->max_t_f_real_ -= hyper_period_;
	}


	AbstractedJob* virtual_start_job = new AbstractedJob("vs_sparse", NULL, 0ULL);

	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); ++sparse_graph_it)
	{
		std::uint64_t the_num_of_predecessors
			= (*sparse_graph_it)->j_prev_det_predecessors_.size()
			+ (*sparse_graph_it)->j_s_det_predecessors_.size()
			+ (*sparse_graph_it)->j_f_det_predecessors_.size()
			+ (*sparse_graph_it)->j_p_det_predecessors_.size();

		if (the_num_of_predecessors == 0 || (*sparse_graph_it)->mapped_task_->physical_read_constraint_ == true)
		{
			if ((*sparse_graph_it)->mapped_task_->physical_read_constraint_ == true)
				virtual_start_job->edge_weights_[(*sparse_graph_it)] = (*sparse_graph_it)->min_t_s_real_;
			else
				virtual_start_job->edge_weights_[(*sparse_graph_it)] = 0ULL;
		}
		
		for (auto succ = (*sparse_graph_it)->j_prev_det_successors_.begin(); succ != (*sparse_graph_it)->j_prev_det_predecessors_.end(); ++succ)
		{
			auto w = (*sparse_graph_it)->edge_weights_.find(*succ);
			if (w == (*sparse_graph_it)->edge_weights_.end())
				(*sparse_graph_it)->edge_weights_[*succ] = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
			else if (w->second < (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_)
				w->second = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
		}
		for (auto succ = (*sparse_graph_it)->j_s_det_successors_.begin(); succ != (*sparse_graph_it)->j_s_det_predecessors_.end(); ++succ)
		{
			auto w = (*sparse_graph_it)->edge_weights_.find(*succ);
			if (w == (*sparse_graph_it)->edge_weights_.end())
				(*sparse_graph_it)->edge_weights_[*succ] = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
			else if (w->second < (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_)
				w->second = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
		}
		for (auto succ = (*sparse_graph_it)->j_f_det_successors_.begin(); succ != (*sparse_graph_it)->j_f_det_predecessors_.end(); ++succ)
		{
			auto w = (*sparse_graph_it)->edge_weights_.find(*succ);
			if (w == (*sparse_graph_it)->edge_weights_.end())
				(*sparse_graph_it)->edge_weights_[*succ] = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
			else if (w->second < (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_)
				w->second = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
		}
		for (auto succ = (*sparse_graph_it)->j_p_det_successors_.begin(); succ != (*sparse_graph_it)->j_p_det_predecessors_.end(); ++succ)
		{
			auto w = (*sparse_graph_it)->edge_weights_.find(*succ);
			if (w == (*sparse_graph_it)->edge_weights_.end())
				(*sparse_graph_it)->edge_weights_[*succ] = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
			else if (w->second < (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_)
				w->second = (*sparse_graph_it)->mapped_task_->c_best_ / simulator_performance_ratio_;
		}

	}

	return virtual_start_job;

}

/**
  * @brief	Assign EPST for each job in sparse graph
	*
	* @param	Virtual start job
	*/
void AbstractedControlSystem::assign_epst(
	AbstractedJob *virtual_start_job)
{
	std::vector<AbstractedJob*> stack;
	std::map<AbstractedJob*, bool> visited;
	
	visited[virtual_start_job] = false;
	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); ++sparse_graph_it)
		visited[*sparse_graph_it] = false;

	topological_sort(virtual_start_job, stack, visited);

	std::map<AbstractedJob*, std::uint64_t> dist;

	dist[virtual_start_job] = 0ULL;
	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); ++sparse_graph_it)
		dist[*sparse_graph_it] = std::numeric_limits<std::uint64_t>::min();
	
	for (auto job_it = stack.rbegin(); job_it != stack.rend(); ++job_it)
	{
		for (auto adj_job_it = (*job_it)->edge_weights_.begin(); adj_job_it != (*job_it)->edge_weights_.end(); ++adj_job_it)
		{
			if (dist[adj_job_it->first] <= dist[(*job_it)] + adj_job_it->second)
				dist[adj_job_it->first] = dist[(*job_it)] + adj_job_it->second;
		}
	}

	for (auto sparse_graph_it = sparse_graph_.begin(); sparse_graph_it != sparse_graph_.end(); ++sparse_graph_it)
		(*sparse_graph_it)->epst_ = dist[(*sparse_graph_it)];
}

/**
  * @brief	Initialize dense graph which ahs both D-edges and ND-edges and no cycle 
	*
	* @return Virtual start job
	*/
AbstractedJob* AbstractedControlSystem::initialize_dense_graph(void)
{
	// Deep copy the 2-HP's jobs
	AbstractedJob* offline_guider_job = *get_offline_guider_job_by_name((*dense_graph_it)->name_);

	auto replacer = [&](std::set<AbstractedJob*>& dense_graph_link, std::set<AbstractedJob*>& offline_guider_link)
	{
		dense_graph_link.clear();
		for (auto offline_guider_it = offline_guider_link.begin(); offline_guider_it != offline_guider_link.end(); ++offline_guider_it)
		{
			if ((*offline_guider_it)->which_period_ == 1) // Only the links among the second HP should be copied
				dense_graph_link.insert(*get_dense_graph_job_by_name((*offline_guider_it)->name_));
		}
	};

	// Copy both of deterministic and non-deterministic edges
	replacer((*dense_graph_it)->j_prev_det_predecessors_, offline_guider_job->j_prev_det_predecessors_);
	replacer((*dense_graph_it)->j_prev_det_successors_, offline_guider_job->j_prev_det_successors_);

	replacer((*dense_graph_it)->j_s_det_predecessors_, offline_guider_job->j_s_det_predecessors_);
	replacer((*dense_graph_it)->j_s_det_successors_, offline_guider_job->j_s_det_successors_);
	replacer((*dense_graph_it)->j_s_nodet_predecessors_, offline_guider_job->j_s_nodet_predecessors_);
	replacer((*dense_graph_it)->j_s_nodet_successors_, offline_guider_job->j_s_nodet_successors_);

	replacer((*dense_graph_it)->j_f_det_predecessors_, offline_guider_job->j_f_det_predecessors_);
	replacer((*dense_graph_it)->j_f_det_successors_, offline_guider_job->j_f_det_successors_);
	replacer((*dense_graph_it)->j_f_nodet_predecessors_, offline_guider_job->j_f_nodet_predecessors_);
	replacer((*dense_graph_it)->j_f_nodet_successors_, offline_guider_job->j_f_nodet_successors_);

	replacer((*dense_graph_it)->j_p_det_predecessors_, offline_guider_job->j_p_det_predecessors_);
	replacer((*dense_graph_it)->j_p_det_successors_, offline_guider_job->j_p_det_successors_);
	replacer((*dense_graph_it)->j_p_nodet_predecessors_, offline_guider_job->j_p_nodet_predecessors_);
	replacer((*dense_graph_it)->j_p_nodet_successors_, offline_guider_job->j_p_nodet_successors_);

	(*dense_graph_it)->t_r_real_ -= hyper_period_;
	(*dense_graph_it)->min_t_s_real_ -= hyper_period_;
	(*dense_graph_it)->max_t_s_real_ -= hyper_period_;
	(*dense_graph_it)->min_t_f_real_ -= hyper_period_;
	(*dense_graph_it)->max_t_f_real_ -= hyper_period_;

	eliminate_cycles();

	AbstractedJob* virtual_start_job = new AbstractedJob("vs_dense", NULL, 0ULL);

	for (auto dense_graph_it = dense_graph_.begin(); dense_graph_it != dense_graph_.end(); ++dense_graph_it)
	{
		std::uint64_t the_num_of_predecessors
			= (*dense_graph_it)->j_prev_det_predecessors_.size()
			+ (*dense_graph_it)->j_s_det_predecessors_.size()
			+ (*dense_graph_it)->j_s_nodet_predecessors_.size()
			+ (*dense_graph_it)->j_f_det_predecessors_.size()
			+ (*dense_graph_it)->j_f_nodet_predecessors_.size()
			+ (*dense_graph_it)->j_p_det_predecessors_.size()
			+ (*dense_graph_it)->j_p_nodet_predecessors_.size();


		if (the_num_of_predecessors == 0 || (*dense_graph_it)->mapped_task_->physical_read_constraint_ == true)
		{
			if ((*dense_graph_it)->mapped_task_->physical_read_constraint_ == true)
				virtual_start_job->edge_weights_[(*dense_graph_it)] = (*dense_graph_it)->max_t_s_real_ + (*dense_graph_it)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else
				virtual_start_job->edge_weights_[(*dense_graph_it)] = (*dense_graph_it)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_prev_det_successors_.begin(); succ != (*dense_graph_it)->j_prev_det_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_s_det_successors_.begin(); succ != (*dense_graph_it)->j_s_det_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_s_nodet_successors_.begin(); succ != (*dense_graph_it)->j_s_nodet_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_f_det_successors_.begin(); succ != (*dense_graph_it)->j_f_det_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_f_nodet_successors_.begin(); succ != (*dense_graph_it)->j_f_nodet_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_p_det_successors_.begin(); succ != (*dense_graph_it)->j_p_det_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}

		for (auto succ = (*dense_graph_it)->j_p_nodet_successors_.begin(); succ != (*dense_graph_it)->j_p_nodet_successors_.end(); ++succ)
		{
			auto w = (*dense_graph_it)->edge_weights_.find(*succ);
			if (w == (*dense_graph_it)->edge_weights_.end())
				(*dense_graph_it)->edge_weights_[*succ] = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
			else if (w->second < (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_)
				w->second = (*succ)->mapped_task_->c_worst_ / simulator_performance_ratio_;
		}
	}

	return virtual_start_job;
}

/**
  * @brief	Eliminate cycles on dense graph until no more cycle is detected
	*/
void AbstractedControlSystem::eliminate_cycles(void)
{
	while (true)
	{
		AbstractedJob *s;
		std::vector<AbstractedJob*> stack;
		std::map<AbstractedJob*, bool> visited;
		
		stack.clear();
		
		for (auto dense_job_it = dense_graph_.begin(); dense_job_it != dense_graph_.end(); ++dense_job_it)
			visited[(*dense_job_it)] = false;
		
		for (auto dense_job_it = dense_graph_.begin(); dense_job_it != dense_graph_.end(); ++dense_job_it)
		{
			if (visited[*dense_job_it] == false && (s = dfs_dense_graph(*dense_job_it, stack, visited)) != NULL)
				break;
		}
		
		if (s == NULL)
			break;
		else
		{
			std::vector<AbstractedJob*>::iterator path_s = std::find(stack.begin(), stack.end(), s);
			std::vector<AbstractedJob*>::iterator path_e = stack.end() - 1;

			//non-deterministic edges
			std::vector<std::pair<AbstractedJob*, AbstractedJob*> > nd_edges;
			
			for (auto path_it = path_s; path_it != path_e; ++path_it)
			{
				AbstractedJob *prev = *(path_it);
				AbstractedJob *succ = *(path_it + 1);

				if (prev->j_s_nodet_successors_.find(succ) != prev->j_s_nodet_predecessors_.end())
					nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>(prev, succ));
				if (prev->j_f_nodet_successors_.find(succ) != prev->j_f_nodet_predecessors_.end())
					nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>(prev, succ));
				if (prev->j_p_nodet_successors_.find(succ) != prev->j_p_nodet_predecessors_.end())
					nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>(prev, succ));
			}

			if ((*path_e)->j_f_nodet_successors_.find((*path_s)) != (*path_e)->j_s_nodet_successors_.end())
				nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>((*path_e), (*path_s)));
			if ((*path_e)->j_f_nodet_successors_.find((*path_s)) != (*path_e)->j_s_nodet_successors_.end())
				nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>((*path_e), (*path_s)));
			if ((*path_e)->j_f_nodet_successors_.find((*path_s)) != (*path_e)->j_s_nodet_successors_.end())
				nd_edges.push_back(std::pair<AbstractedJob*, AbstractedJob*>((*path_e), (*path_s)));

			std::uint64_t max_diff = 0ULL;
			AbstractedJob *max_diff_prev;
			AbstractedJob *max_diff_succ;
			for(auto edge_it = nd_edges.begin(); edge_it != nd_edges.end(); ++edge_it)
			{
				AbstractedJob *prev = edge_it->first;
				AbstractedJob *succ = edge_it->second;

				std::uint64_t diff = prev->max_t_s_real_ - succ->min_t_s_real_;
				if (diff >= max_diff)
				{
					max_diff = diff;
					max_diff_prev = prev;
					max_diff_succ = succ;
				}
			}

			max_diff_prev->j_s_nodet_successors_.erase(max_diff_succ);
			max_diff_prev->j_f_nodet_successors_.erase(max_diff_succ);
			max_diff_prev->j_p_nodet_successors_.erase(max_diff_succ);
			max_diff_succ->j_s_nodet_predecessors_.erase(max_diff_prev);
			max_diff_succ->j_f_nodet_predecessors_.erase(max_diff_prev);
			max_diff_succ->j_p_nodet_predecessors_.erase(max_diff_prev);
		}
	}
}

/**
  * @brief	Depth-First-Search on dense graph
	*
	*
	* @param	root			root of DFS forest
	* @param	stack			stack for tracing
	* @param	visited		visited flags
	*
	* @return start job of detected cycle
	*/
AbstractedJob* AbstractedControlSystem::dfs_dense_graph(
	AbstractedJob* root,
	std::vector<AbstractedJob*>& stack,
	std::map<AbstractedJob*, bool>& visited)
{
	visited[root] = true;
	stack.push_back(root);

	AbstractedJob *ret;

	for (auto succ = root->j_prev_det_successors_.begin(); succ != root->j_prev_det_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_s_det_successors_.begin(); succ != root->j_s_det_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_s_nodet_successors_.begin(); succ != root->j_s_nodet_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_f_det_successors_.begin(); succ != root->j_f_det_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_f_nodet_successors_.begin(); succ != root->j_f_nodet_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_p_det_successors_.begin(); succ != root->j_p_det_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	for (auto succ = root->j_p_nodet_successors_.begin(); succ != root->j_p_nodet_predecessors_.end(); ++succ)
	{
		if (visited[*succ] == false && (ret = dfs_dense_graph(*succ, stack, visited)) != nullptr)
			return ret; //sucessor인데, dfs간 결과 null값이 나오지 않는 경우 -> 잘 들어갔음 no cycle  
		else if (std::find(stack.begin(), stack.end(), *succ) != stack.end()) //ret null 값인 경우, pop했기 때문에 successor가 stack안에 존재하지 않는 경우, return *succ
			return *succ;
	}

	stack.pop_back();

	return NULL;

}

/**
  * @brief	Assign LPFT for each job in dense graph
	*
	* @param	Virtual Start job
	*/
void AbstractedControlSystem::assign_lpft(
	AbstractedJob *virtual_start_job)
{
	std::vector<AbstractedJob*> stack;
	std::map<AbstractedJob*, bool> visited;

	visited[virtual_start_job] = false;
	for (auto dense_graph_it = dense_graph_.begin(); dense_graph_it != dense_graph_.end(); ++dense_graph_it)
		visited[*dense_graph_it] = false;

	topological_sort(virtual_start_job, stack, visited);
	
	std::map<AbstractedJob*, std::uint64_t> dist;

	dist[virtual_start_job] = 0ULL;
	for (auto dense_graph_it = dense_graph_.begin(); dense_graph_it != dense_graph_.end(); ++dense_graph_it)
		dist[*dense_graph_it] = std::numeric_limits<std::uint64_t>::min();

	for (auto job_it = stack.rbegin(); job_it != stack.rend(); ++job_it)
	{
		for (auto adj_job_it = (*job_it)->edge_weights_.begin(); adj_job_it != (*job_it)->edge_weights_.end(); ++adj_job_it)
		{
			if (dist[adj_job_it->first] <= dist[(*job_it)] + adj_job_it->second)
				dist[adj_job_it->first] = dist[(*job_it)] + adj_job_it->second;
			
		}
	}

	for (auto dense_graph_it = dense_graph_.begin(); dense_graph_it != dense_graph_.end(); ++dense_graph_it)
		(*dense_graph_it)->lpft_ = dist[(*dense_graph_it)];
}





