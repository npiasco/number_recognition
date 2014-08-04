import rospy
import sys

#fixe the fact that it don't find the right smach repository by itself
sys.path.insert(1,'/opt/strands/strands_catkin_ws/src/executive_smach')
import smach
import smach_ros

import cv2
from geometrical_perspective_recovery import gpr
import openCV_treatment as treat_tool
from image_compare import ImageComparator
from treatment_error import TreatmentError
import recognition_state_machine as RSM


class GPR_treatment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
								outcomes=['succeed', 'fail'],
								input_keys=['im_input'],
								output_keys=['im_output'])
								
	def execute(self, userdata):

		try:
			im=gpr(userdata.im_input)
			userdata.im_output=im
			cv2.imshow('GPR', im)
			cv2.waitKey(5)
		except:
			#Modify gpr to raise TreatmentError
			return 'fail'
		else:
			return 'succeed'
		

def recognizeNumber(im):

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['succeed', 'aborted'])
	sm.userdata.first_im=im

	# Open the container
	with sm:
		# Create recognition state machine
		sm_recognition = smach.StateMachine(outcomes=['succeed', 'fail', 'fail_after_gpr'],
											input_keys=['im_input_machine'])
		with sm_recognition:
			sm_recognition.userdata.im=None
			sm_recognition.userdata.floor_number=None
			
			smach.StateMachine.add('Color_Extraction', RSM.Color_Extraction(), 
									transitions={'extracted':'Number_Extraction', 
												'fail_after_gpr':'fail_after_gpr', 
												'fail':'fail'},
									remapping={'im_input':'im_input_machine',
												'im_output':'im'})
			smach.StateMachine.add('Number_Extraction', RSM.Number_Extraction(),
									transitions={'succeed':'Recognition',
												'fail':'Color_Extraction'},
									remapping={'im_input':'im',
												'im_output':'im'})
			smach.StateMachine.add('Recognition', RSM.Recognition(),
									transitions={'succeed':'succeed',
												'fail':'Binary_Treatment'},
									remapping={'im_input':'im',
												'im_output':'im',
												'number':'floor_number'})													
			smach.StateMachine.add('Binary_Treatment', RSM.Binary_Treatment(),
									transitions={'succeed':'Recognition',
												'fail':'Color_Extraction'},
									remapping={'im_input':'im',
												'im_output':'im'})
		
		smach.StateMachine.add('Recognition_Process', sm_recognition, 		
								transitions={'fail':'GPR',
											'fail_after_gpr':'aborted', 
											'succeed':'succeed'},
								remapping={'im_input_machine':'first_im'})
								
		smach.StateMachine.add('GPR', GPR_treatment(), 		
								transitions={'fail':'aborted', 
											'succeed':'Recognition_Process'},
								remapping={'im_input':'first_im', 
											'im_output':'first_im'})


	# Create and start the introspection server to visualize the state machine
	sis = smach_ros.IntrospectionServer('rnumber', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()



	if outcome=='aborted':
		raise TreatmentError('State machine aborted', 'nc')
	else:
		return sm_recognition.userdata.floor_number
	
	
	
	
	
	
	
	
	
	
	