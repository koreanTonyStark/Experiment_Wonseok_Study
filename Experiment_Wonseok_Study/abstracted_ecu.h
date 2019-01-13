#ifndef __ABSTRACTED_ECU_H__
#define __ABSTRACTED_ECU_H__

#include <string>
#include <vector>
#include <cstdint>

class AbstractedTask;

class AbstractedEcu
{
public:
	const std::string            name_;
	const std::string            sched_policy_;
	std::vector<AbstractedTask*> pended_tasks_;
	
	AbstractedEcu(void);
	AbstractedEcu(std::string name, std::string sched_policy);
	//copy-constructor
	AbstractedEcu(const AbstractedEcu& ref);
	~AbstractedEcu(void);
	

	
};

#endif