import rospy
import sys

#fixe the fact that it don't find the right smach repository by itself
sys.path.insert(1,'/opt/strands/strands_catkin_ws/src/executive_smach')
import smach
import smach_ros

import cv2
#from geometrical_perspective_recovery import gpr
import geometrical_perspective_recovery as GPR
import openCV_treatment as treat_tool
from image_compare import ImageComparator
from treatment_error import TreatmentError
import recognition_state_machine as RSM


"""
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
		
"""
def recognizeNumber(im):

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['succeed', 'aborted'])
	sm.userdata.first_im=im

	# Open the container
	with sm:
		# Create recognition state machine
		sm_recognition = smach.StateMachine(outcomes=['succeed', 'fail', 'fail_after_gpr'],
											input_keys=['im_input_machine'])
	
		sm_recognition.userdata.im_input_machine=None
		sm_recognition.userdata.im=None
		sm_recognition.userdata.floor_number=None
		
		with sm_recognition:
			
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
								transitions={'fail':'GPR_Treatment',
											'fail_after_gpr':'aborted', 
											'succeed':'succeed'},
								remapping={'im_input_machine':'first_im'})
			
		# Create GPR state machine
		sm_gpr = smach.StateMachine(outcomes=['succeed', 'fail'],
											input_keys=['im_input'],
											output_keys=['im_output'])
		sm_gpr.userdata.im=None									
		sm_gpr.userdata.im_input=None
		sm_gpr.userdata.im_output=None
		with sm_gpr:

			smach.StateMachine.add('Color_Extraction', GPR.Color_Extraction(), 		
									transitions={'fail':'fail', 
												'extracted':'GPR'},
									remapping={'im_input':'im_input', 
												'im_output':'im'})
	
			smach.StateMachine.add('GPR', GPR.GPR(), 		
									transitions={'fail':'Color_Extraction', 
												'succeed':'succeed'},
									remapping={'im_input':'im_input', 
												'im_extracted':'im',
												'im_output':'im_output'})
												
		smach.StateMachine.add('GPR_Treatment', sm_gpr, 		
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
	
	
	
	
	
	
	
	
	
	
	
