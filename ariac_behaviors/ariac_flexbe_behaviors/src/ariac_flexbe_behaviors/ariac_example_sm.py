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
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_behaviors.notify_shipment_ready_sm import notify_shipment_readySM
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
		self.add_behavior(notify_shipment_readySM, 'notify_shipment_ready')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		names = ['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		move_group = 'manipulator'
		gripper = 'arm1_vacuum_gripper_link'
		move_goup_prefix = '/ariac/arm1'
		# x:106 y:247, x:331 y:369
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.joint_values = []
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.part = 'gasket_part'
		_state_machine.userdata.offset = 0.1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:44 y:21
			OperatableStateMachine.add('MoveR1Home',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='home', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'DetectCameraPart', 'planning_failed': 'WaitRetry1', 'control_failed': 'WaitRetry1', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:187 y:22
			OperatableStateMachine.add('DetectCameraPart',
										DetectPartCameraState(ref_frame='arm1_linear_arm_actuator', camera_topic='/ariac/logical_camera_1', camera_frame='logical_camera_1_frame', time_out=5.0),
										transitions={'continue': 'PartPoseMessage', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'part': 'part', 'pose': 'part_pose'})

			# x:638 y:24
			OperatableStateMachine.add('ComputePick',
										ComputeGraspState(move_group=move_group, move_group_prefix=move_goup_prefix, offset=0.04, joint_names=names, tool_link='ee_link', rotation=0),
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
										transitions={'reached': 'WachtEven', 'planning_failed': 'WaitRetry3', 'control_failed': 'WaitRetry3'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:502 y:25
			OperatableStateMachine.add('MoveR1PreGrasp1',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='bin3PreGrasp', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'ComputePick', 'planning_failed': 'WaitRetry2', 'control_failed': 'WaitRetry2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:802 y:337
			OperatableStateMachine.add('MoveR1PreGrasp2',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='bin3PreGrasp', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'MoveR1PreDrop', 'planning_failed': 'WaitRetry4', 'control_failed': 'WaitRetry4', 'param_error': 'failed'},
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

			# x:634 y:493
			OperatableStateMachine.add('ComputeDrop',
										ComputeGraspState(move_group=move_group, move_group_prefix=move_goup_prefix, offset=-0.2, joint_names=names, tool_link='ee_link', rotation=0),
										transitions={'continue': 'JointValuesMessage_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:463 y:490
			OperatableStateMachine.add('JointValuesMessage_2',
										MessageState(),
										transitions={'continue': 'JointNamesValues_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'joint_values'})

			# x:297 y:493
			OperatableStateMachine.add('JointNamesValues_2',
										MessageState(),
										transitions={'continue': 'MoveR1ToDrop'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'joint_names'})

			# x:101 y:493
			OperatableStateMachine.add('MoveR1ToDrop',
										MoveitToJointsDynAriacState(move_group_prefix=move_goup_prefix, move_group=move_group, action_topic='/move_group'),
										transitions={'reached': 'notify_shipment_ready', 'planning_failed': 'WaitRetry6', 'control_failed': 'WaitRetry6'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:798 y:492
			OperatableStateMachine.add('GetAgvPose',
										GetObjectPoseState(object_frame='kit_tray_1', ref_frame='arm1_linear_arm_actuator', time_out=5.0),
										transitions={'continue': 'ComputeDrop', 'time_out': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'time_out': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:73 y:90
			OperatableStateMachine.add('WaitRetry1',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1Home'},
										autonomy={'done': Autonomy.Off})

			# x:513 y:93
			OperatableStateMachine.add('WaitRetry2',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreGrasp1'},
										autonomy={'done': Autonomy.Off})

			# x:953 y:188
			OperatableStateMachine.add('WaitRetry3',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1ToPick'},
										autonomy={'done': Autonomy.Off})

			# x:945 y:325
			OperatableStateMachine.add('WaitRetry4',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreGrasp2'},
										autonomy={'done': Autonomy.Off})

			# x:121 y:574
			OperatableStateMachine.add('WaitRetry6',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1ToDrop'},
										autonomy={'done': Autonomy.Off})

			# x:980 y:421
			OperatableStateMachine.add('WaitRetry5',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreDrop'},
										autonomy={'done': Autonomy.Off})

			# x:805 y:407
			OperatableStateMachine.add('MoveR1PreDrop',
										ariac_flexbe_states__SrdfStateToMoveit(config_name='tray1PreDrop', move_group=move_group, move_group_prefix=move_goup_prefix, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'GetAgvPose', 'planning_failed': 'WaitRetry5', 'control_failed': 'WaitRetry5', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:65 y:338
			OperatableStateMachine.add('notify_shipment_ready',
										self.use_behavior(notify_shipment_readySM, 'notify_shipment_ready'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
