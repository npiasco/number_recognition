import rospy
import sys

#fixe the fact that it don't find the right smach repository by itself
sys.path.insert(1,'/opt/strands/strands_catkin_ws/src/executive_smach')
import smach
import smach_ros

from treatment_error import TreatmentError
import recognition_state_machine as RSM
from luminosity_correction import Luminosity_Correction
from template_recognition import Template_Recognition


def recognizeNumber(im):

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['succeed', 'aborted'])
	sm.userdata.first_im=im
	sm.userdata.template_im=im.copy()
	sm.userdata.number=None
	# Open the container
	with sm:


		# Create recognition state machine
		sm_recognition = smach.StateMachine(outcomes=['succeed', 'fail', 'fail_after_LC'],
						    input_keys=['im_input_machine'],
						    output_keys=['floor_number'])
	
		sm_recognition.userdata.im_input_machine=None
		sm_recognition.userdata.im=None
		sm_recognition.userdata.floor_number=None
		
		with sm_recognition:
			
			#Binarization of the image to obtein only red pixels (the number on the lift)
			smach.StateMachine.add('Color_Extraction', RSM.Color_Extraction(), 
									transitions={'extracted':'Number_Extraction', 
												'fail_after_LC':'fail_after_LC', 
												'fail':'fail'},
									remapping={'im_input':'im_input_machine',
												'im_output':'im'})
			#Resize the image on the number to proceed recognition
			smach.StateMachine.add('Number_Extraction', RSM.Number_Extraction(),
									transitions={'succeed':'Recognition',
												'fail':'Color_Extraction'},
									remapping={'im_input':'im',
												'im_output':'im'})
			#Recognition of the number
			smach.StateMachine.add('Recognition', RSM.Recognition(),
									transitions={'succeed':'succeed',
												'fail':'Binary_Treatment'},
									remapping={'im_input':'im',
												'im_output':'im',
												'number':'floor_number'})													
			#Morphological transformation on the number if the recognition failed
			smach.StateMachine.add('Binary_Treatment', RSM.Binary_Treatment(),
									transitions={'succeed':'Recognition',
												'fail':'Color_Extraction'},
									remapping={'im_input':'im',
												'im_output':'im'})
		
		smach.StateMachine.add('Recognition_Process', sm_recognition,
				       transitions={'fail':'Luminosity_Correction',
						    'fail_after_LC':'Template_Recognition',
						    'succeed':'succeed'},
				       remapping={'im_input_machine':'first_im',
						  'floor_number':'number'})
			
		# Create luminosity correction state
		smach.StateMachine.add('Luminosity_Correction', Luminosity_Correction(),
				       transitions={'aborted':'Template_Recognition',
						   'succeed':'Recognition_Process'},
				       remapping={'im_input':'first_im',
						  'im_output':'first_im'})

	        # Create template matching recognition state
		smach.StateMachine.add('Template_Recognition', Template_Recognition(),
				       transitions={'fail':'aborted',
						   'succeed':'succeed'},
				       remapping={'im_input':'template_im',
						  'number':'number'})

		




	# Create and start the introspection server to visualize the state machine
	sis = smach_ros.IntrospectionServer('rnumber', sm, '/SM_ROOT')
	sis.start()


	# Start a timer
	t=rospy.get_rostime()
	while t==0:
		t=rospy.get_rostime()

	# Execute the state machine
	outcome = sm.execute()

	elaps=rospy.get_rostime()-t
	rospy.loginfo("Treatment time :%i s %i ns"%(elaps.secs,elaps.nsecs))
	if outcome=='aborted':
		raise TreatmentError('State machine cannot find the right number', 'end of state machine')
	else:
		return sm.userdata.number
	
	
	
	
	
	
	
	
	
	
	
