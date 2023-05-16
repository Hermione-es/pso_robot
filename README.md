#PSO 기반 멀티로봇 군집지능 알고리즘

자율이동로봇은 자신의 위치와 장애물을 이용하여 경로를 생성하는 경로 계획(Path Planning)이 필수적이다. 그러나 단일 로봇으로 얻을 수 있는 데이터는 한정적이고 복잡한 환경에서 그 성능은 크게 떨어질 수 있다. PSO(Particle Swarm Optimization) 알고리즘을 기반으로 복수의 로봇을 사용하여 로봇이 빠르게 최적의 경로를 찾도록 하고자 한다. 

--------------------
1. 멀티로봇 가제보 시뮬레이션

    roslaunch pso_robot simulation.launch

2. PSO 기반 군집 주행
    
    roslaunch pso_robot pso_swarm
    roslaunch pso_robot pso_swarm
    
