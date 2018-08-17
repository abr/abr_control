"""
Move the UR5 VREP arm to a target position.
The simulation ends after 1.5 simulated seconds, and the
trajectory of the end-effector is plotted in 3D.
"""
import numpy as np
import traceback

from abr_control.controllers import OSC
from abr_control.controllers import path_planners
from abr_control.interfaces import VREP
from abr_control.utils import DataHandler
import timeit
import time

class JacoTest():
    def __init__(self):
        self.dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
        self.save_name = 'ur5_joint_difference_3'

    def run_sim(self):
        print('Starting simulation...')
        # initialize our robot config
        if 'jaco' in self.save_name:
            from abr_control.arms import jaco2 as arm
            robot_config = arm.Config(use_cython=True, hand_attached=True)
        elif 'ur5' in self.save_name:
            from abr_control.arms import ur5 as arm
            robot_config = arm.Config(use_cython=True)

        joints = []
        n_targets = 10
        for ii in range(0,n_targets):
            joints_temp = self.dat.load(params=['q'],
                    save_location='1lb_random_target/pd_no_weight_23/session000/run%03d'%ii)['q']
            joints.append(np.copy(joints_temp[::10]))

        # create our VREP interface
        print(np.array(joints).shape)
        interface = VREP(robot_config, dt=0.04)
        interface.connect()

        # set up lists for tracking data
        calc_track = []
        actual_track = []
        try:
            feedback = interface.get_feedback()
            for nn in range(0, n_targets):
                current_run_joints = joints[nn]
                print('current_run_joints shape: ', np.array(current_run_joints).shape)
                print('target %i of %i'%(nn, n_targets))
                for jj, qs in enumerate(current_run_joints):
                    # move arm to starting position
                    interface.send_target_angles(q=qs)
                    # move the simulation one time step
                    interface.step_sim()
                    # get the end-effector's initial position
                    feedback = interface.get_feedback()

                    step_actual = []
                    step_calc = []
                    for ii in range(0,7):
                        if ii == 6:
                            joint = 'EE'
                        else:
                            joint = 'joint%i'%ii
                        current_joint_calc = robot_config.Tx(joint, q=feedback['q'])
                        interface.set_xyz(name='target', xyz=current_joint_calc)
                        current_joint_act = interface.get_xyz(name=joint)
                        step_actual.append(current_joint_act)
                        step_calc.append(current_joint_calc)
                    actual_track.append(np.copy(step_actual))
                    #print(step_actual[4])
                    calc_track.append(np.copy(step_calc))
                    if jj % 10 == 0:
                        print('%i / %i'%(jj, len(current_run_joints)))
                        print('Actual: ', step_actual)
                        print('Calculated: ', step_calc)
                #print(actual_track)
        except:
            print(traceback.format_exc())

        finally:
            # stop and reset the VREP simulation
            interface.disconnect()

            print('Simulation terminated...')

            overwrite = True
            create = True
            loc = 'simulations/%s/session000/run000'%(self.save_name)

            params = {'actual_joints': actual_track, 'calculated_joints': calc_track}
            self.dat.save(data=params,
                    save_location=loc, overwrite=overwrite, create=create)

            custom_params = {'Notes': """
            Previous sim did not work, need to step through simulation to get
            feedback. Added step function to vrep interface. Had to turn off
            dynamics and np.copy the actual joint positions since they were
            a constant value otherwise.
            """}
            self.dat.save(data=custom_params,
                    save_location='simulations/%s/test_parameters'%self.save_name, overwrite=overwrite,
                    create=create)

    def calc_things(self):
        data = self.dat.load(params=['actual_joints', 'calculated_joints'],
                save_location='simulations/%s/session000/run000'%self.save_name)
        actual = data['actual_joints']
        calculated = data['calculated_joints']
        print('calculated: ', np.array(calculated).shape)
        errors = []
        # go through all time steps
        for ii in range(0, len(actual)):
            step_calc = calculated[ii]
            step_act = actual[ii]
            if ii == 0:
                print('calc step: ', np.array(step_calc).shape)
            step_error = 0
            # go through each joint and sum errors
            for jj in range(0,len(step_act)):
                errs = np.sqrt(np.sum((step_calc[jj] - step_act[jj])**2))
                step_error += errs
            errors.append(step_error)
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(errors)
        plt.title('Forward Kinematic Error: %s'%self.save_name)
        plt.show()


        print('actual: ', actual.shape)
        print('calculated', calculated.shape)
        #joint_n = 4
        plt.figure()
        for joint_n in range(0, len(step_act)):
            actual_n_joint = actual[:,joint_n,:].T
            calculated_n_joint = calculated[:,joint_n,:].T
            print('actual: ', actual_n_joint.shape)
            print('calculated', calculated_n_joint.shape)
            print(np.ceil(len(step_act)/3))
            print(1+joint_n*3)
            print(len(step_act))
            plt.subplot(3,np.ceil(len(step_act)),1+joint_n)
            plt.title(self.save_name)
            plt.xlabel('X : joint%i'%(joint_n))
            plt.plot(actual_n_joint[0], 'r', label='actual')
            plt.plot(calculated_n_joint[0], 'b--', label='calculated')
            plt.legend()
            plt.subplot(3,np.ceil(len(step_act)),8+joint_n)
            plt.xlabel('Y: joint%i'%joint_n)
            plt.plot(actual_n_joint[1], 'r', label='actual')
            plt.plot(calculated_n_joint[1], 'b--', label='calculated')
            plt.legend()
            plt.subplot(3,np.ceil(len(step_act)),15+joint_n)
            plt.xlabel('Z: joint%i'%joint_n)
            plt.plot(actual_n_joint[2], 'r', label='actual')
            plt.plot(calculated_n_joint[2], 'b--', label='calculated')
            plt.legend()
        plt.show()

if __name__ == '__main__':
    runsim = JacoTest()
    #runsim.run_sim()
    runsim.calc_things()
