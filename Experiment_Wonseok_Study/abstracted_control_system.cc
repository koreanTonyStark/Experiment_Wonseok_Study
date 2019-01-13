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
				//lexicographic order·Î return 
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
	//ToDay 
	
}
