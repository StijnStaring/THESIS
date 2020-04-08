- This folder is used to generate the offline data of the expert planner doing lane keeping.
- The simulation can be launched by the main.m file.
- line_data1.mat contains the high curvature track'
- FunctionGeneration.m implements the optimization problem in CasADi syntax and generates the CasADi Function Object (M) that is then loaded in the main before the simulation, and called by PlanningFunction.m during the simulation
- The optimization problem is solved every 100 ms. With this rate, and by commenting the plot blocks in the Simulink file, the simulation is almost real-time.
  It is anyway possible to change the rate by:
            - Right Click on the Motion Planning Subsystem
            - Select Block Parameters (Subsystem)
            - Change the Sample Time