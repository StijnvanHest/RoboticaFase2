#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.srdf_state_to_moveit import SrdfStateToMoveit as ariac_flexbe_states__SrdfStateToMoveit
from ariac_flexbe_states.detect_part_camera_state import DetectPartCameraState
from ariac_flexbe_states.compute_grasp_state import ComputeGraspState
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 16 2020
@author: Gerard Harkema
'''
class ariac_exampleSM(Behavior):
	'''
	Voorbeeld statemachine voor de ariac 2019 opdracht
	'''


	def __init__(self):
		super(ariac_exampleSM, self).__init__()
		self.name = 'ariac_example'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		names = ['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		move_group = 'manipulator'
		gripper = 'arm1_vacuum_gripper_link'
		move_goup_prefix = '/ariac/arm1'
		# x:859 y:433, x:331 y:369
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.joint_values = []
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.part = 'gasket_part'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:44 y:21
			OperatableStateMachine.add('MoveR1Home',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='home', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'DetectCameraPart', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:187 y:22
			OperatableStateMachine.add('DetectCameraPart',
										DetectPartCameraState(ref_frame='world', camera_topic='/ariac/logical_camera_1', camera_frame='logical_camera_1_frame', time_out=5.0),
										transitions={'continue': 'PartPoseMessage', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'part': 'part', 'pose': 'part_pose'})

			# x:638 y:24
			OperatableStateMachine.add('ComputePick',
										ComputeGraspState(move_group=move_group, move_group_prefix=move_goup_prefix, offset=0.0, joint_names=names, tool_link='ee_link', rotation=0),
										transitions={'continue': 'JointValuesMessage', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:786 y:26
			OperatableStateMachine.add('JointValuesMessage',
										MessageState(),
										transitions={'continue': 'JointNamesValues'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'joint_values'})

			# x:797 y:109
			OperatableStateMachine.add('JointNamesValues',
										MessageState(),
										transitions={'continue': 'MoveR1ToPick'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'joint_names'})

			# x:756 y:187
			OperatableStateMachine.add('MoveR1ToPick',
										MoveitToJointsDynAriacState(move_group_prefix=move_goup_prefix, move_group=move_group, action_topic='/move_group'),
										transitions={'reached': 'WachtEven', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:502 y:25
			OperatableStateMachine.add('MoveR1PreGrasp1',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='bin3PreGrasp', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'ComputePick', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:802 y:337
			OperatableStateMachine.add('MoveR1PreGrasp2',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='bin3PreGrasp', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:830 y:258
			OperatableStateMachine.add('WachtEven',
										WaitState(wait_time=2),
										transitions={'done': 'MoveR1PreGrasp2'},
										autonomy={'done': Autonomy.Off})

			# x:356 y:24
			OperatableStateMachine.add('PartPoseMessage',
										MessageState(),
										transitions={'continue': 'MoveR1PreGrasp1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'part_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
