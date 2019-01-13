#include <algorithm>

#include "abstracted_ecu.h"
#include "abstracted_task.h"

AbstractedEcu::AbstractedEcu(void)
	:
	name_("NOT INITIALIZED"),
	sched_policy_("NOT INITIALIZED")
{
	//do nothing
}

AbstractedEcu::AbstractedEcu(
	std::string name,
	std::string sched_policy)
	:
	name_(name),
	sched_policy_(sched_policy)
{

}

AbstractedEcu::AbstractedEcu(
	const AbstractedEcu& ref)
	:
	name_(ref.name_),
	sched_policy_(ref.sched_policy_)
{
	pended_tasks_.resize(ref.pended_tasks_.size());
	std::copy(ref.pended_tasks_.begin(), ref.pended_tasks_.end(), pended_tasks_.begin());
}

AbstractedEcu::~AbstractedEcu(void)
{
	// Do nothing
}