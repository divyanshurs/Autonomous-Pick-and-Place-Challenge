import sys
import numpy as np
from copy import deepcopy

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import FK
from lib.solveIK import IK
from lib.rrt import rrt
from lib.loadmap import loadmap
import time

import tf
import geometry_msgs.msg
tf_broad  = tf.TransformBroadcaster()
def show_pose(H,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(H),
        tf.transformations.quaternion_from_matrix(H),
        rospy.Time.now(),
        frame,
        "base"
    )

def show_pose_world(H,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(H),
        tf.transformations.quaternion_from_matrix(H),
        rospy.Time.now(),
        frame,
        "world_frame"
    )

def static_tags(tag_array):
    static_list_tags=["tag1","tag2", "tag3", "tag4", "tag5", "tag6"]
    static_list = [e for e in tag_array if(e[0] in static_list_tags)]
    print("static_list")
    print(static_list)
    static_array = np.array(static_list)
    return static_array



if __name__ == "__main__":

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    arm.safe_move_to_position(arm.neutral_position()) # on your mark!
    team_num = 0

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
        team_num = 1
    else:
        print("**  RED TEAM  **")
    print("****************")
    kk = input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print(kk)
    print("Go!\n") # go!

    # STUDENT CODE HERE

    # Detect some tags...
    # for (name, pose) in detector.get_detections():
    #      print(name,'\n',pose)

    # Move around...
    tag_array = detector.get_detections()
    print(tag_array)


    index=0
    for idx,tags in enumerate(tag_array):
        if(tags[0]=="tag0"):
            index=idx

    tag0_name = tag_array[index][0] 
    tag0_pose = tag_array[index][1]
    tag0_pose = np.array(tag0_pose) #calibaration tag

    static_array = static_tags(tag_array)
    print("static_array")
    print(static_array)
    print(static_array.shape[0])


    #cal magic mat (0 is the calibaration tag)
    identity = np.identity(3)
    translation = [-0.5, 0,0,1]
    T_b0 = np.zeros((4,4)) #base ke respect me calibration tag
    T_b0[:-1, :-1] = identity
    T_b0[:, 3] = translation

    T_0c = np.linalg.inv(tag0_pose) #calibration tag ke respect me camera 

    T_bc = np.matmul(T_b0, T_0c) #CONSTANT HAI. Base ke respect me camera

    #cal T_bw assuming RED
    if(team_num == 0):
        T_wb = [[1.0, 0, 0, 0],  #world ke respect me base
                [0,  1.0, 0 ,-0.978],
                [ 0,  0 ,1.00000000e+00 , 0.0],
                [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]
    else:
        T_wb = [[1.0, 0, 0, 0],  #world ke respect me base
                [0,  1.0, 0 ,0.978],
                [ 0,  0 ,1.00000000e+00 , 0.0],
                [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]        

    #show_pose_world(T_wb, "T_wb")

    T_bw = np.linalg.inv(T_wb)

    tag_dict = {
    "tag6": [[0,-1,0,0],
             [1,0,0,0],
             [0,0,1,-0.025],
             [0,0,0,1]],

    "tag3": [[0,-1,0,0],
             [0,0,1,0],
             [-1,0,0,-0.025],
             [0,0,0,1]],

    "tag4": [[1,0,0,0],
             [0,0,1,0],
             [0,-1,0,-0.025],
             [0,0,0,1]],

    "tag2": [[-1,0,0,0],
             [0,0,1,0],
             [0,1,0,-0.025],
             [0,0,0,1]],

    "tag1": [[0,1,0,0],
             [0,0,1,0],
             [1,0,0,-0.025],
             [0,0,0,1]],

    "tag5": [[0,1,0,0],
             [1,0,0,0],
             [0,0,-1,-0.015],
             [0,0,0,1]]
    }

    if(team_num ==0):
        T_wg_static =  [[1.0, 0, 0, 0.64],#0.562 #0.48
                         [0,  1, 0 , -0.76], #-0.74
                         [ 0,  0 ,1.00000000e+00 , 0.225],
                         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]
        T_wg_dynamic =  [[1.0, 0, 0, 0.53],#0.562 #0.48
                         [0,  1, 0 , -0.74],
                         [ 0,  0 ,1.00000000e+00 , 0.225],
                         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]

    else:
        T_wg_static =  [[1.0, 0, 0, 0.64],#0.562
                         [0,  1, 0 , 0.76],#0.74
                         [ 0,  0 ,1.00000000e+00 , 0.225],
                         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]
        T_wg_dynamic =  [[1.0, 0, 0, 0.53],#0.562 #0.48
                         [0,  1, 0 , 0.74],
                         [ 0,  0 ,1.00000000e+00 , 0.225],
                         [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00  ,1.00000000e+00]]
       
    #show_pose_world(T_wg_static, "T_wg_static")
    T_bg = np.matmul(T_bw, T_wg_static)

    # T_bg = np.matmul(T_bw, T_wg_dynamic)


    #visualize

    for show in range(static_array.shape[0]):
        tag_to_display_name = static_array[show][0]
        tag_to_display_name = tag_to_display_name + str(show)
        tag_to_display_matrix = static_array[show][1]
        tag_to_display_matrix = np.matmul(T_bc, tag_to_display_matrix)
        show_pose(tag_to_display_matrix, tag_to_display_name)




    #GOAL PE JANA HAI TO STACK
    T_180_Y = [[np.cos(np.pi), 0, np.sin(np.pi), 0], #rotate a given frame by 180 deg wrt Y
         [0,1,0, 0],
         [-np.sin(np.pi), 0, np.cos(np.pi),0],
         [0,0,0,1]]

    T_90_Z = [[np.cos(np.pi/2), -np.sin(np.pi/2), 0, 0], #rotate a given frame by 90 deg wrt Z
         [np.sin(np.pi/2), np.cos(np.pi/2),0,0],
         [0,0,1,0],
         [0,0,0,1]]

    T_180_Z = [[np.cos(np.pi), -np.sin(np.pi), 0, 0], #rotate a given frame by 90 deg wrt Z
     [np.sin(np.pi), np.cos(np.pi),0,0],
     [0,0,1,0],
     [0,0,0,1]]

    T_white_down = [[0, 0, -1, 0], #composite rotation for final orientation
             [0, -1, 0,0],
             [-1, 0, 0,0],
             [0,0,0,1]]

    T_90_X = [[1, 0, 0, 0], #rotate 90 about X
             [0, 0, -1,0],
             [0, 1, 0,0],
             [0,0,0,1]]

    T_minus_90_X = [[1, 0, 0, 0], #rotate 90 about X
             [0, 0, 1,0],
             [0, -1, 0,0],
             [0,0,0,1]]    

    T_180_X = [[1, 0, 0, 0], #rotate 180 about X
             [0, -1, 0,0],
             [0, 0, -1,0],
             [0,0,0,1]]
    #for z-down figure out the 4 cases and the corrosponding rot mat for all and then multiply a rotation accordingly
    #TAG OF INTEREST    
    count = 0
    for i in static_array:
        # continue
       
        #arm.safe_move_to_position(arm.neutral_position()) # on your mark!
        count +=1
        tag1_pose = i[1]

        T_bn = np.matmul(T_bc, tag1_pose) #Base ke respect me tag pose

        #BLOCK KE UPAR
        T_bn_z_up = deepcopy(T_bn) #first pose to reach before grasping 
        z_tol = 0.2
        T_bn_z_up[2][3] = T_bn_z_up[2][3] + z_tol
        T_bn_z_up = np.matmul(T_bn_z_up, T_180_Y) #Transform1
        #show_pose(T_bn_z_up, "T_bn_z_up_180Y")
        T_bn_z_up = np.matmul(T_bn_z_up, T_90_Z) #Transform2
        #show_pose(T_bn_z_up, "T_bn_z_up_90Z")

        if(i[0] == 'tag5'): #TAG5
            # T_bn_z_up = np.matmul(T_bn_z_up, T_white_down) #Transform3 only if tag5 is up
            # T_bn_z_up = np.matmul(T_bn_z_up, T_90_X) #Transform3 only if tag5 is up
            i[0] = 'tag6'  #WIERD THING TO AVOID WHITE DOWN CONDITION
            show_pose(T_bn_z_up, "T_bn_z_up_90X")


        ik = IK()
        show_pose(T_bn_z_up, "T_bn_z_up")
        path_to_save = ''
        if(team_num ==0):
            q_T_bn_z_up_seed = [[-0.19577597, -0.04491164, -0.21444899 ,-1.80687921, -0.00973376 , 1.7629861,-0.42665383],
                 [-0.02529456, -0.16543661, -0.16204773, -1.94089839, -0.0271512 ,  1.77751653 ,-1.07898526],
                 [-0.08025832 ,-0.17941268, -0.13385946, -1.95649504, -0.02434219,  1.77859375 ,-0.39250309]]
            path_to_save = 'q_T_bn_z_up_seed_red'
            #q_T_bn_z_up_seed = np.loadtxt(path_to_save)
        else:
            print("going to q_T_bn_z_up_seed")
            q_T_bn_z_up_seed = [[ 0.04130792 ,-0.19131081 , 0.14002457, -1.96875834  ,0.02712844 , 1.77920076 ,0.6844679 ],
            [ 0.18200538 ,-0.0256165  , 0.22435643 ,-1.78457253 , 0.00580163 , 1.75959467, 2.03236395],
            [-0.12091825 , 0.08145533, -0.02984555, -2.07068698 , 1.71300179 , 1.77905912 ,-1.38128761],
            [-0.05074705 ,-0.16319507 , 0.24874002 ,-1.93508447 , 0.04087191  ,1.77666002 ,2.35873439]]
            path_to_save = 'q_T_bn_z_up_seed_blue'
            np.savetxt(path_to_save,q_T_bn_z_up_seed)

            #q_T_bn_z_up_seed = np.loadtxt(path_to_save)


        q_T_bn_z_up_seed = np.array(q_T_bn_z_up_seed)


        for zz in q_T_bn_z_up_seed:
            q, success, rollout = ik.inverse(T_bn_z_up, zz)
            #q, success, rollout = ik.inverse(T_bn_z_up, arm.neutral_position())
            print("q_T_bn_z_up_seed")
            print(q)
            if(success == True):
                print("SUCCESS!!!! q_T_bn_z_up")
                print(q)
                arm.safe_move_to_position(q)
                if(np.where(q == q_T_bn_z_up_seed)[0].shape[0] ==0):
                    new_array = np.vstack((q_T_bn_z_up_seed, q))
                    #np.savetxt(path_to_save,new_array)
                    print("q was diff pushed")
                break

        Z_rot_flag = False 
        if(success == False):
            q, success, rollout = ik.inverse(T_bn_z_up, arm.neutral_position())
            if(success == False):
                print("T_bn_z_up ROTATED BY 180Z")
                Z_rot_flag = True
                T_bn_z_up = np.matmul(T_bn_z_up, T_180_Z)
                show_pose(T_bn_z_up, "T_bn_z_up_180Z")
                #q, success, rollout = ik.inverse(T_bn_z_up, arm.neutral_position())
                
                for zz_4 in q_T_bn_z_up_seed:
                    q, success, rollout = ik.inverse(T_bn_z_up, zz_4)
                    #q, success, rollout = ik.inverse(T_bn_z_up, arm.neutral_position())
                    print("q_T_bn_z_up_seed")
                    print(q)
                    if(success == True):
                        print("SUCCESS!!!! q_T_bn_z_up")
                        print(q)
                        arm.safe_move_to_position(q)
                        if(np.where(q == q_T_bn_z_up_seed)[0].shape[0] ==0):
                            new_array = np.vstack((q_T_bn_z_up_seed, q))
                            #np.savetxt(path_to_save,new_array)
                            print("q was diff pushed")
                        break                


                if(success == False):
                    sys.exit("Bro dikkat hai q_T_bn_z_up_seed")
                # else: 
                #     arm.safe_move_to_position(q)
            else:
                arm.safe_move_to_position(q)

            
        


        #TO GRASP
        arm.open_gripper()

        T_bn_pursue = deepcopy(T_bn)
        # if(i[0] == 'tag5'): #TAG5
        #     T_bn_pursue[2][3] = T_bn_pursue[2][3] - 0.015
        # else:
        #     T_bn_pursue[2][3] = T_bn_pursue[2][3] - 0.025

        T_bn_pursue[2][3] = T_bn_pursue[2][3] - 0.025 #TAG5
        
        T_bn_pursue = np.matmul(T_bn_pursue, T_180_Y) #Transform1
        T_bn_pursue = np.matmul(T_bn_pursue, T_90_Z)  #Transform2
        if(Z_rot_flag == True):
            T_bn_pursue = np.matmul(T_bn_pursue, T_180_Z)
        # if(i[0] == 'tag5'): #TAG5
        #     T_bn_pursue = np.matmul(T_bn_pursue, T_white_down)  #Transform3 only if tag5 is up

        ik = IK()
        print("GRIP KAR RHA AB")
        #show_pose(T_bn_pursue, "T_bn_pursue")
        #show_pose(T_bn, "T_bn")
        q, success, rollout = ik.inverse(T_bn_pursue, q)
        arm.safe_move_to_position(q)

        arm.exec_gripper_cmd(0.048, 2) #GRIP KAR LIYA


        T_before_z_up = deepcopy(T_bn_pursue)
        T_before_z_up[:, -1] = np.array([T_bn_pursue[0][3], T_bn_pursue[1][3], T_bn_pursue[2][3]+0.3,1])
        #T_before_z_up[:, -1] = np.array([T_bn_pursue[0][3], T_bn_pursue[1][3], T_bn_pursue[2][3]+0.05*(count),1])
        q_goal_before_z_up, success, rollout = ik.inverse(T_before_z_up, q)
        arm.safe_move_to_position(q_goal_before_z_up)

        T_b_block_centre = np.matmul(T_bn, np.array(tag_dict[i[0]]))
        show_pose(T_b_block_centre, "T_b_block_centre")
        T_block_centre_b = np.linalg.inv(T_b_block_centre)
        T_block_centre_bn_pursue = np.matmul(T_block_centre_b, T_bn_pursue)
        print("T_block_centre_bn_pursue")
        print(T_block_centre_bn_pursue)
        T_goal_z_up = np.identity(4) 
        T_goal_z_up[:,-1]= T_bg[:,-1] #BLOCK CENTER KA FINAL POSITION YE HOGA
        T_goal_z_up[2][3] = T_goal_z_up[2][3] + 0.04*count                      #GOAL POSITION HERE
        #show_pose(T_goal_z_up, "T_goal_z_up"+str(count))


        T_b_end_effector = np.matmul(T_goal_z_up, T_block_centre_bn_pursue)
        #CHECKING FOR FINAL END EFFECTOR ORIENATATION TO AVOID SELF COLLISION

        dot_z = np.dot(T_b_end_effector[:-1, 2], [1,0,0])
        dot_y = np.dot(T_b_end_effector[:-1, 1], [0,1,0])
        #if(dot_z<-0.1 and dot_y >0.1) :
        if(dot_z<-0.1) :
            T_b_end_effector = np.matmul(T_b_end_effector, T_minus_90_X)
            print("CHECKING FOR FINAL END EFFECTOR ORIENATATION TO AVOID SELF COLLISION")
            print(dot_z)
            # dot_z_2 = np.dot(T_b_end_effector[:-1, 2], [1,0,0])
            # if(dot_z_2<0):
            #     T_b_end_effector = np.matmul(T_b_end_effector, T_minus_90_X)
            #     print("CHECKING FOR FINAL END EFFECTOR ORIENATATION TO AVOID SELF COLLISION")






        show_pose(T_b_end_effector, "T_b_end_effector"+str(count))
        if(team_num ==0):     
            q_T_b_end_effector_seed = np.array([[-0.94975309 ,-1.66979592,  1.47263796 ,-2.5118378 ,2.72386943 ,2.79728177, -0.36645582],
                [ 0.10842604 ,-0.14774171  ,0.24316856 ,-2.04781983 , 0.03751746 , 1.90414671 ,1.12216403],
                [-0.95973442 ,-1.56521291 , 1.35259917 ,-2.48409895 , 2.58964885 , 2.80299195, -0.39023493], #IMP
              [-2.24013391, -0.75936612 , 2.03854145 ,-2.07145848,  1.96862671 , 1.02390719 ,-1.59845951],
              [-2.20924873 ,-0.71127328 , 2.00046868 ,-2.0770058 ,  1.94929825,  1.03981602 ,-1.5632624 ],
                [-0.94975309 ,-1.66979592 , 1.47263796 ,-2.5118378  , 2.72386943 , 2.79728177 ,-0.36645582]])
            #np.savetxt('q_T_b_end_effector_seed_red',q_T_b_end_effector_seed)
            #q_T_b_end_effector_seed = np.loadtxt('q_T_b_end_effector_seed_red')
            #q_T_b_end_effector_seed = np.loadtxt('q_T_b_end_effector_seed')


        else:
            print("going hereree q_T_b_end_effector_seed")
            q_T_b_end_effector_seed = np.array([[ 1.33567787, -1.21665021 ,-1.62756829, -1.821897  ,  0.32946235,  1.55360962 ,-0.92357595],
                [ -1.33567787, -1.21665021 ,-1.62756829, -1.821897  ,  0.32946235,  1.55360962 ,-0.92357595],
                [ 1.07972778, -0.92550974, -1.53199891 ,-2.77444591 ,-0.34314968  ,1.97563484 ,0.05197495],
                [-1.6121716  , 0.73695892 ,-1.47646659 ,-2.54648169 ,-0.54650461 ,-1.5125525 ,1.64887326],
                [-0.13333994,  0.26668008 ,-0.32449408, -2.16574293 , 2.59015214 , 2.17608144 ,-1.92629385],
                [ 1.23863987, -1.18324659 ,-1.61942986, -2.40743744, -0.32953218 , 3.69124179, -0.14836865],
                [-0.06113145 , 0.10687666, -0.26461298, -2.09713485 , 2.62843849 , 2.43555725 ,-1.92167758],
                [-0.31949871, -0.13639457, -0.06653688, -2.38749258 , 2.56687093 , 2.37794715,-1.9283846 ],
                [-0.20304521, -0.14498236, -0.15397437, -2.047912  , -0.02345377 , 1.90453621, 0.43765321]])
            #np.savetxt('q_T_b_end_effector_seed_blue',q_T_b_end_effector_seed)
            #q_T_b_end_effector_seed = np.loadtxt('q_T_b_end_effector_seed_blue')



        q_T_b_end_effector_seed = np.array(q_T_b_end_effector_seed)


        for zz1 in q_T_b_end_effector_seed:
            q, success, rollout = ik.inverse(T_b_end_effector, zz1)
            #q, success, rollout = ik.inverse(T_b_end_effector, arm.neutral_position())
            print("q_T_b_end_effector_seed")
            print(q)
            if(success == True):
                print("SUCCESS!!!! q_T_b_end_effector_seed")
                print(q)
                arm.safe_move_to_position(q)
                if(np.where(q == q_T_b_end_effector_seed)[0].shape[0] ==0):
                    new_array = np.vstack((q_T_b_end_effector_seed, q))
                    # if(team_num ==0):
                    #     #np.savetxt('q_T_b_end_effector_seed_red',new_array)
                    # else:
                    #     #np.savetxt('q_T_b_end_effector_seed_blue',new_array)

                    print("q was diff pushed")
                break

        if(success == False):
            #q, success, rollout = ik.inverse(T_b_end_effector, q_goal_before_z_up)
            q, success, rollout = ik.inverse(T_b_end_effector, arm.neutral_position())

            if(success==False):
                print("ROTATED BY 90")
                T_b_end_effector = np.matmul(T_b_end_effector, T_90_X)
                dot_z_2 = np.dot(T_b_end_effector[:-1, 2], [1,0,0])
                if(dot_z_2<0):
                    T_b_end_effector = np.matmul(T_b_end_effector, T_90_X)
                    print("CHECKING FOR FINAL END EFFECTOR ORIENATATION TO AVOID SELF COLLISION")
                
                show_pose(T_b_end_effector, "T_b_end_effector"+str(count))
                #q, success, rollout = ik.inverse(T_b_end_effector, arm.neutral_position())

                for zz22 in q_T_b_end_effector_seed:
                    q, success, rollout = ik.inverse(T_b_end_effector, zz22)
                    #q, success, rollout = ik.inverse(T_b_end_effector, arm.neutral_position())
                    print("q_T_b_end_effector_seed")
                    print(q)
                    if(success == True):
                        print("SUCCESS!!!! q_T_b_end_effector_seed")
                        print(q)
                        arm.safe_move_to_position(q)
                        if(np.where(q == q_T_b_end_effector_seed)[0].shape[0] ==0):
                            new_array = np.vstack((q_T_b_end_effector_seed, q))
                            # if(team_num ==0):
                            #     #np.savetxt('q_T_b_end_effector_seed_red',new_array)
                            # else:
                            #     #np.savetxt('q_T_b_end_effector_seed_blue',new_array)

                            print("q was diff pushed")
                        break

                if(success==False):
                    sys.exit("Bro dikkat hai in q_T_b_end_effector_seed")
                # else:
                #     arm.safe_move_to_position(q)
            else:
                arm.safe_move_to_position(q)


        # q, success, rollout = ik.inverse(T_b_end_effector, q_T_b_end_effector_seed)
        # print("q_T_b_end_effector")
        # print(q)

        # arm.safe_move_to_position(q)

        arm.open_gripper()


        T_final = deepcopy(T_b_end_effector)
        T_final[2][3] = T_final[2][3] + 0.1
        q, success, rollout = ik.inverse(T_final, q)
        arm.safe_move_to_position(q)




        # dynamic

    # print(arm.get_gripper_state())
    
    if(team_num == 0):
        T_winterest =[[1, 0, 0, 0], 
             [0, 1, 0,-0.275], #0.305
             [0, 0, 1,0.25], #0.2
             [0,0,0,1]] 


        q_bn_seed = [ 0.83055982  ,0.05106262  ,0.75614345 ,-1.83715602 ,-0.0367034   ,1.87412654 ,0.81162191]
        q_bn_seed_1 = [ 1.235854 ,   0.44906426  ,0.44855844 ,-1.31070836 ,-0.19148189 , 1.71767541 ,0.88791932]
    else:
        T_winterest =[[1, 0, 0, 0], 
             [0, 1, 0,0.275], #0.305
             [0, 0, 1,0.25], #0.2
             [0,0,0,1]] 


        q_bn_seed = [-1.27780143 ,0.52531682 ,-0.40107262 ,-1.31561964  ,0.20247786  ,1.80118764,-0.89088202]

        

        # q_bn_seed_1 = [ 1.235854 ,   0.44906426  ,0.44855844 ,-1.31070836 ,-0.19148189 , 1.71767541 ,0.88791932]




    
    T_bn = np.matmul(T_bw, T_winterest)
    

    T_bg = np.matmul(T_bw, T_wg_dynamic)


    #BLOCK KE UPAR
    T_bn_z_up = deepcopy(T_bn) #first pose to reach before grasping 
    z_tol = 0.1
    x_tol = 0.1
    T_bn_z_up[2][3] = T_bn_z_up[2][3] + z_tol
    #T_bn_z_up[0][3] = T_bn_z_up[0][3] - x_tol
    T_bn_z_up = np.matmul(T_bn_z_up, T_180_Y) #Transform1
    
    if(team_num==1):
        T_bn_z_up = np.matmul(T_bn_z_up, T_180_Z) #Transform2
    

    ik = IK()
    
    show_pose(T_bn_z_up, "T_bn_z_up")
    
    q_start, success, rollout = ik.inverse(T_bn_z_up,q_bn_seed)
    # print(q_start)
   


    T_w_goal_mat = deepcopy(np.array(T_winterest))
    translation = np.array([0,-0.275,0.225,1])
    
    if(team_num==1):
        translation = np.array([0,0.275,0.225,1])
    
    T_w_goal_mat[:, 3] = translation
    T_b_goal_mat = np.matmul(T_bw, T_w_goal_mat)
    

    T_b_goal_mat = np.matmul(T_b_goal_mat, T_180_Y) #Transform1

    if(team_num==1):
        T_b_goal_mat = np.matmul(T_b_goal_mat, T_180_Z) #Transform2
    
    
   
    print("T_b_goal_mat")
    print(T_b_goal_mat)
    show_pose(T_b_goal_mat, "T_b_goal_mat")
    q_goal, success, rollout = ik.inverse(T_b_goal_mat, q_start)

    T_place_goal = deepcopy(T_bg)
    T_place_goal = np.matmul(T_place_goal, T_180_Y)
    q_place, success, rollout = ik.inverse(T_place_goal, arm.neutral_position())


    #TAG OF INTEREST    
    count = 0
    number_of_tries = 800
    for i in range(number_of_tries):
        arm.safe_move_to_position(q_start)
        arm.open_gripper()

        arm.safe_move_to_position(q_goal)
        
        arm.exec_gripper_cmd(0.048, 2) #GRIP KAR LIYA
        
        print(arm.get_gripper_state())

        if(arm.get_gripper_state()['position'][0]>0.035):
            T_place_goal = deepcopy(T_bg)
            T_place_goal = np.matmul(T_place_goal, T_180_Y)
            T_place_goal[2][3] = T_place_goal[2][3] + 0.04*count
            show_pose(T_place_goal,"goal for placing"+ str(count))

            #q_place, success, rollout = ik.inverse(T_place_goal, arm.neutral_position())
            for seeds in q_T_b_end_effector_seed:
                q_place, success, rollout = ik.inverse(T_place_goal, seeds)
                if(success == True):
                    arm.safe_move_to_position(q_place)
                    arm.open_gripper()
                    count = count+1
                    break
        else:
            time.sleep(2)






   






















     