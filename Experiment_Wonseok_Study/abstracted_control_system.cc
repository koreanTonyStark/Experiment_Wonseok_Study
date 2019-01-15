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
		if (worse_case_execution_window_[mapped_ecu->name_][wcbp_start_real_it].compare("EMPTY") == 0)
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
	
	for(std::uint64_t core =0; )






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



















