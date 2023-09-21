//
// Created by lucas on 21.09.23.
//

/**
In this code we want to generate and plan the tasks for the demo. A task could consist for example in "bring me the spice".
This means a task is comprised of a series of subtasks which need to be executed in a certain order. Lets call this an action.
An action could be "go pick", which is comprised of a movement and a grasp. It could also include a forcing or a place.
 We will call these "basic block".
Like this we can build a structure:

    -Task
        - action1 (move, grasp, force)
        - action2
        - ....
Between the actions there must be some trigger or logical dependency, depending on external factor. Now when we look at an action which can include
any combination of moving, forcing, grasping and placing. They can be parametrized comletely in the following manner:

    -move
        -position
        -orientation
        -precision + velocity i.e K, D (we will leave this constant with whatever is programmed in the controller)
        -move end effector 0/1
            -if move: velocity, width
    -forcing
        -force magnitude
        -direction
        -evtl. duration
   -grasp
        -finger velocity
        -evtl. special orientation
        - there is no other component, since the force cannot be controlled
   -place
        -finger velocity

We add a special building block "IDLE", in which the end effector waits for external forcing in free floating mode
    -idle
        - Force threshold
        - task after force threshold is reached
        - impedance (normally 0)

The use case should contain following activities to show the human robot collaboration:

1) The robot prepares the scene together with the human. The human gets spices while the robots gets the tools or vice versa
   (optionally implement logic to get one after the other, and move to the next if nothing is there)
   -> Safety bubble

   Task 1: arrange set of objects
        for each object do:
            action1 = go pick
            action2 = go place
        Return SUCCESS when all objects are placed
2) Robot gets idle and into free flow mode. Wait until human has cut the veggies. The human then pushes the robot to get the robot to move to the bowl.
   -> smart impedance

   Task 2: wait and move to bowl when prompted
           action3 = wait for human Force input
           if F > threshold && displacement > threshold:
                action = go pick (the bowl)
                action = move to coordinate (pour bowl)
                action = place (put dish to wash)
           Return SUCCESS after bowl is empty (duration constraint)
           Return to IDLE

   The human then debarasses the surface, while robot pours ingredients into pan. The robot puts down the dish near soap (and gets idle), which the human has to get
   to simultaneously, to spray the now empty table.
   -> safety bubble
3) Human gives robot another push, the robot starts cleaning the surface, while the human stars cooking

    Task 3: wait and clean surface when prompted
            if F, displacement > threshold:
                action = move to coordinate (start of cleaning trajectory)
                while trajectory is not completed:
                    action = move (to next waypoint) && action = forcing
                action = go pick (cleaning towel)
                action = go place (cleaning towel)
            Return SUCCESS when whole trajectory is cleaned and towel is in place
   -> Force control

END od use case
**/
/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    return 0;
}