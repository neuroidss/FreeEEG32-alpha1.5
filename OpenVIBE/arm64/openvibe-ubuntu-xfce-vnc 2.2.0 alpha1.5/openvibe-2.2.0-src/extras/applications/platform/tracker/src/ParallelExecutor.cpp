
#include <functional>

#include <iostream>

#include <system/ovCTime.h>

#include "ParallelExecutor.h"

//___________________________________________________________________//
//                                                                   //

using namespace std;
using namespace OpenViBETracker;

//___________________________________________________________________//
//                                                                   //

bool ParallelExecutor::initialize(const uint32_t nThreads)
{
	m_Quit = false;
	m_nJobsRunning = 0;
	m_JobList.clear();

	// ExecutorView passes the working threads a few function handles to the executor that holds the state
	ExecutorView ctx(*this);

	for(uint32_t i=0;i<nThreads;i++)
	{
		m_WorkerThreads.push_back(new CWorkerThread());
		m_Threads.push_back(new std::thread(std::bind(&CWorkerThread::startWorkerThread, m_WorkerThreads[i], ctx, i)));
	}

	return true;
}

bool ParallelExecutor::uninitialize() 
{
	// Tell the threads waiting in the cond its time to quit, don't care of pending jobs
	{ // scope for lock
		std::unique_lock<std::mutex> lock(m_JobMutex);	
		m_JobList.clear();
		m_Quit = true;
	}

	m_HaveWork.notify_all();

	for(uint32_t i=0;i<m_Threads.size();i++)
	{
		m_Threads[i]->join();

		delete m_Threads[i];
		delete m_WorkerThreads[i];
	}

	m_Threads.clear();
	m_WorkerThreads.clear();	

	return true;
}

bool ParallelExecutor::pushJob(const jobCall& someJob)
{
	// @fixme to add better concurrency, push a list instead; lock();add list;unlock();notify_all();
	{ // lock scope
		std::lock_guard<std::mutex> lock(m_JobMutex);

		m_JobList.push_back(someJob);
	}

	m_HaveWork.notify_one();

	return true;
}


bool ParallelExecutor::pushJobList(const std::deque<jobCall>& vJobList)
{
	{ // lock scope
		std::lock_guard<std::mutex> lock(m_JobMutex);

		if(m_JobList.size() > 0) {
			std::cout << "Error, trying to push list with old jobs pending\n";
			return false;
		}

		m_JobList = vJobList;
	}

	m_HaveWork.notify_all();


	return true;
}

bool ParallelExecutor::waitForAll()
{
	std::unique_lock<std::mutex> lock(m_JobMutex);
	while(m_JobList.size()>0) {
		m_JobDone.wait(lock); 
	}

	return true;
}

bool ParallelExecutor::getJob(jobCall& job) {
	// Wait until we get a job or are told to quit
	std::unique_lock<std::mutex> lock(m_JobMutex);

	m_HaveWork.wait(lock, [this](void){ return (this->m_Quit || this->m_JobList.size() > 0); } );

	if(m_Quit) {
		return false;
	}

	// Ok, we have a job
	job = m_JobList.front();
	m_JobList.pop_front();

	m_nJobsRunning++;

	return true;
};

bool ParallelExecutor::declareDone(void)
{
	std::unique_lock<std::mutex> lock(m_JobMutex);	

	m_nJobsRunning--;	
	m_JobDone.notify_one();

	return true;
};

bool ParallelExecutor::clearPendingJobs(void)								
{
	std::unique_lock<std::mutex> lock(m_JobMutex);	
	m_JobList.clear(); 
	return true;
};           

size_t ParallelExecutor::getJobCount(void) const 
{ 
	std::unique_lock<std::mutex> lock(m_JobMutex);	
	return m_JobList.size();
};

bool ParallelExecutor::isIdle(void) const 
{
	std::unique_lock<std::mutex> lock(m_JobMutex);	
	return (m_JobList.size() == 0 && m_nJobsRunning ==0);
}


//___________________________________________________________________//
//                                                                   //

void testFunction(void* data) {
	for(uint32_t i=0;i<10;i++) {
		std::cout << "Fun: " << *(uint32_t*)(data) << "\n";
		System::Time::sleep(*(uint32_t*)data);
	}
//	return true;
}

bool ParallelExecutor::launchTest(void)
{
 	int stuff[6] = {500,666,50,1000,300,100};
	
	std::cout << "Push test\n";

	this->pushJob(std::bind(testFunction, &stuff[0]));
	this->waitForAll();
	this->pushJob(std::bind(testFunction, &stuff[1]));
	this->pushJob(std::bind(testFunction, &stuff[2]));
	this->pushJob(std::bind(testFunction, &stuff[3]));
	this->pushJob(std::bind(testFunction, &stuff[4]));
	this->pushJob(std::bind(testFunction, &stuff[2]));
	this->waitForAll();
	this->waitForAll();

	std::cout << "Pushlist test\n";

	std::deque<jobCall> jobList;
	jobList.push_back(std::bind(testFunction, &stuff[0]));
	jobList.push_back(std::bind(testFunction, &stuff[1]));
	this->pushJobList(jobList);
	this->waitForAll();

	std::cout << "Done\n";

	return true;
}


