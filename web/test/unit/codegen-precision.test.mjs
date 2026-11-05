import assert from 'node:assert/strict';
import test from 'node:test';
import { formatCall, pyRepr } from '../../src/api/codegen.js';

test('finite float literals retain precision and round-trip at small magnitudes', () => {
  for (const value of [0.12345678901234566, 1e-7, 5e-324]) {
    const literal = pyRepr(value, 'float');
    assert.equal(Number(literal), value);
    assert.doesNotMatch(literal, /\.0$/);
  }
});

test('float hints produce valid integer-scale exponential and signed-zero literals', () => {
  assert.equal(pyRepr(1e21, 'float'), '1e+21');
  assert.equal(pyRepr(1e20, 'float'), `${String(1e20)}.0`);
  assert.equal(pyRepr(-0, 'float'), '-0.0');
  assert.equal(pyRepr(-0), '-0.0');
});

test('precision is preserved through hinted calls and nested ROS messages', () => {
  const tiny = 1e-7;
  assert.equal(formatCall('robot.move_to_pose', [], { position: [tiny, 0.12345678901234566, 0], quat_xyzw: [1, 0, 0, 0] }),
    'robot.move_to_pose(position=[1e-7, 0.12345678901234566, 0.0], quat_xyzw=[1.0, 0.0, 0.0, 0.0])');
  const message = pyRepr({ __class__: 'Point', x: tiny, y: 1e21, z: -0 });
  assert.equal(message, 'geometry_msgs.msg.Point(x=1e-7, y=1e+21, z=-0.0)');
});
