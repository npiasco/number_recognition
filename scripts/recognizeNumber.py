import rospy
import sys

#fixe the fact that it don't find the right smach repository by itself
sys.path.insert(1,'/opt/strands/strands_catkin_ws/src/executive_smach')
import smach
import smach_ros

from treatment_error import TreatmentError
import recognition_state_machine as RSM
from luminosity_correction import Luminosity_Correction


def recognizeNumber(im):

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['succeed', 'aborted'])
	sm.userdata.first_im=im

	# Open the container
	with sm:
		# Create recognition state machine
		sm_recognition = smach.StateMachine(outcomes=['succeed', 'fail', 'fail_after_LC'],
						    input_keys=['im_input_machine'])
	
		sm_recognition.userdata.im_input_machine=None
		sm_recognition.userdata.im=None
		sm_recognition.userdata.floor_number=None
		
		with sm_recognition:
			
			smach.StateMachine.add('Color_Extraction', RSM.Color_Extraction(), 
									transitions={'extracted':'Number_Extraction', 
												'fail_after_LC':'fail_after_LC', 
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
				       transitions={'fail':'Luminosity_Correction',
						    'fail_after_LC':'aborted',
						    'succeed':'succeed'},
				       remapping={'im_input_machine':'first_im'})
			
		# Create luminosity correction state
		smach.StateMachine.add('Luminosity_Correction', Luminosity_Correction(),
				       transitions={'aborted':'aborted',
						   'succeed':'Recognition_Process'},
				       remapping={'im_input':'first_im',
						  'im_output':'first_im'})




	# Create and start the introspection server to visualize the state machine
	#sis = smach_ros.IntrospectionServer('rnumber', sm, '/SM_ROOT')
	#sis.start()


	# Start a timer
	t=rospy.get_rostime()
	while t==0:
		t=rospy.get_rostime()

	# Execute the state machine
	outcome = sm.execute()

	elaps=rospy.get_rostime()-t
	rospy.loginfo("Treatment time :%i s %i ns"%(elaps.secs,elaps.nsecs))
	if outcome=='aborted':
		raise TreatmentError('State machine aborted', 'nc')
	else:
		return sm_recognition.userdata.floor_number
	
	
	
	
	
	
	
	
	
	
	
