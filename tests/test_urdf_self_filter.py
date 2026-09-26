from pathlib import Path
import sys
import math
from unittest.mock import patch
import numpy as np
import pytest
import xacro
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.urdf_filter_geometry import RobotGeometry


def test_articulated_link_moves_without_erasing_gap_or_old_position():
    xml = '''<robot name="test"><link name="base_link"/>
    <link name="leg"><visual><origin xyz="0.3 0 0"/><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
    <joint name="hip" type="revolute"><parent link="base_link"/><child link="leg"/>
    <origin xyz="0 0.3 0"/><axis xyz="0 0 1"/></joint></robot>'''
    model = RobotGeometry(xml, lambda _: None, padding=.01)
    points = np.array([[.3, .3, 0], [0, .6, 0], [0, 0, 0], [.3, .37, 0], [.3, .3, -.1]])
    assert model.contains(points, {'hip': 0}).tolist() == [True, False, False, False, False]
    assert model.contains(points, {'hip': math.pi / 2}).tolist() == [False, True, False, False, False]
    with pytest.raises(ValueError, match='Missing joint'):
        model.contains(points, {})


def test_real_go2_visual_meshes_load_and_leave_gap_below_trunk():
    root = Path(__file__).resolve().parents[1]
    # Expand the tracked model so a clean CI checkout runs this test too.
    description = root / 'go2_description'
    with patch('ament_index_python.packages.get_package_share_directory', return_value=str(description)):
        urdf = xacro.process_file(str(description / 'xacro/robot.xacro')).toxml()
    model = RobotGeometry(urdf, lambda _: description)
    joints = {f'{leg}_{joint}_joint': angle for leg in ('FL', 'FR', 'RL', 'RR')
              for joint, angle in (('hip', 0.), ('thigh', .8), ('calf', -1.5))}
    assert len(model.required_joints) == 12
    assert len(model.parts) >= 25
    # In the trunk vs open air below it; floor point is not body-wide masked.
    assert model.contains(np.array([[0., 0., 0.], [0., 0., -.3], [0., .5, 0.]]), joints).tolist() == [True, False, False]


@pytest.mark.parametrize('geometry', ['<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>', '<cylinder radius="0.1" length="0.2"/>'])
def test_shadow_removes_occluded_return_but_keeps_visible_and_far_points(geometry):
    xml = f'<robot name="test"><link name="base_link"><visual><origin xyz="0.5 0 0"/><geometry>{geometry}</geometry></visual></link></robot>'
    model = RobotGeometry(xml, lambda _: None, padding=0)
    points = np.array([[.8, 0, 0], [.2, 0, 0], [.8, .5, 0], [2, 0, 0]])
    assert model.contains(points, {}).tolist() == [False]*4
    assert model.shadows(points, {}, np.zeros(3), 1.).tolist() == [True, False, False, False]
    assert not model.shadows(points, {}, np.array([.5, 0, 0]), 1.).any()


def test_shadow_moves_with_articulated_link():
    xml = '''<robot name="test"><link name="base_link"/><link name="leg"><visual><origin xyz="0.5 0 0"/><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link><joint name="hip" type="revolute"><parent link="base_link"/><child link="leg"/><axis xyz="0 0 1"/></joint></robot>'''
    model = RobotGeometry(xml, lambda _: None, padding=0)
    points = np.array([[.8, 0, 0], [0, .8, 0]])
    assert model.shadows(points, {'hip': 0}, np.zeros(3)).tolist() == [True, False]
    assert model.shadows(points, {'hip': math.pi/2}, np.zeros(3)).tolist() == [False, True]
