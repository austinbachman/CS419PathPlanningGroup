/* By: Austin Bachman */
/* For CS491 Aerial Robotics */
/* Kostas Alexis, University of Nevada, Reno */

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

#include <fstream>

using namespace ompl;

struct config_data
{
    std::string env_file;
    double start_X,
           start_Y,
           start_Z,
           goal_X,
           goal_Y,
           goal_Z;
};

void benchmark0(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count, config_data cfg)
{
    benchmark_name = std::string("quadrotor");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/quadrotor.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + cfg.env_file;
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(cfg.start_X);
    start->setY(cfg.start_Y); 
    start->setZ(cfg.start_Z);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(cfg.goal_X);
    goal->setY(cfg.goal_Y);
    goal->setZ(cfg.goal_Z);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35; cs[1] = 35; cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 25;
}

void quadcopterPlannerData(app::SE3RigidBodyPlanning& setup)
{
    std::cout<<"\n\n***** Planning with RRTstar *****\n" << std::endl;
    setup.setPlanner(std::make_shared<geometric::RRTstar>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(10.))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        geometric::PathGeometric& path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        setup.clear();
    }
    else
    {
        std::cout<<"\n***** No solution found *****\n" << std::endl;
    }

    std::cout<<"\n\n***** Planning with RRT *****\n" << std::endl;
    setup.setPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(10.))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        geometric::PathGeometric& path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        setup.clear();
    }
    else
    {
        std::cout<<"\n***** No solution found *****\n" << std::endl;
    }

    std::cout<<"\n\n***** Planning with KPIECE1 *****\n" << std::endl;
    setup.setPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(10.))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        geometric::PathGeometric& path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        setup.clear();
    }
    else
    {
        std::cout<<"\n***** No solution found *****\n" << std::endl;
    }

    std::cout<<"\n\n***** Planning with EST *****\n" << std::endl;
    setup.setPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(10.))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        geometric::PathGeometric& path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        setup.clear();
    }
    else
    {
        std::cout<<"\n***** No solution found *****\n" << std::endl;
    }

    std::cout<<"\n\n***** Planning with PRM *****\n" << std::endl;
    setup.setPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(10.))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        geometric::PathGeometric& path(setup.getSolutionPath());
        path.printAsMatrix(std::cout);
        setup.clear();
    }
    else
    {
        std::cout<<"\n***** No solution found *****\n" << std::endl;
    }
}

void preRunEvent(const base::PlannerPtr& /*planner*/)
{
}

void postRunEvent(const base::PlannerPtr& /*planner*/, tools::Benchmark::RunProperties& /*run*/)
{
}

int main(int argc, char **argv)
{
    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;
    std::ifstream fin;
    std::string config_file_name;
    config_data cfg_data;

    config_file_name = argv[argc-1];

    fin.open(config_file_name);
    if( !fin )
    {
        std::cout << "error opening configuration file\n";
        return -1;
    }

    fin >> cfg_data.env_file
        >> cfg_data.start_X
        >> cfg_data.start_Y
        >> cfg_data.start_Z
        >> cfg_data.goal_X
        >> cfg_data.goal_Y
        >> cfg_data.goal_Z;

    fin.close();

    benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count, cfg_data);

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr& planner) { preRunEvent (planner); });
    b.setPostRunEvent([](const base::PlannerPtr& planner, tools::Benchmark::RunProperties& run)
        { postRunEvent(planner, run); });

    b.addPlanner(std::make_shared<geometric::RRTstar>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    int sampler_id = argc > 2 ? ((argv[1][0] - '0') % 4) : 0;

    if (sampler_id == 0 || sampler_id < 0)
    {
        // run all planners with a uniform valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
            {
                return std::make_shared<base::UniformValidStateSampler>(si);
            });
        b.setExperimentName(benchmark_name + "_uniform_sampler");

        std::cout<<"\n***** Planning for environment: " << cfg_data.env_file << " *****\n";

        b.benchmark(request);
        b.saveResultsToFile();
        quadcopterPlannerData(setup);
    }

    if (sampler_id == 1 || sampler_id < 0)
    {
        // run all planners with a Gaussian valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
            {
                return std::make_shared<base::GaussianValidStateSampler>(si);
            });
        b.setExperimentName(benchmark_name + "_gaussian_sampler");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 2 || sampler_id < 0)
    {
        // run all planners with a obstacle-based valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
            {
                return std::make_shared<base::ObstacleBasedValidStateSampler>(si);
            });
        b.setExperimentName(benchmark_name + "_obstaclebased_sampler");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 3 || sampler_id < 0)
    {
        // run all planners with a maximum-clearance valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
            {
                auto vss = std::make_shared<base::MaximizeClearanceValidStateSampler>(si);
                vss->setNrImproveAttempts(5);
                return vss;
            });
        b.setExperimentName(benchmark_name + "_maxclearance_sampler");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    return 0;
}
