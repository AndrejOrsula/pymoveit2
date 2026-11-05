// The robots the demo simulates. `?robot=<id>` picks one; the Panda is the default.

import { OPEN_MANIPULATOR_X } from './open_manipulator_x.js';
import { PANDA } from './panda.js';

export const ROBOTS = Object.freeze({ [PANDA.id]: PANDA, [OPEN_MANIPULATOR_X.id]: OPEN_MANIPULATOR_X });

/** The robot with this id, else the Panda. */
export function robotById(id) {
  return Object.hasOwn(ROBOTS, id ?? '') ? ROBOTS[id] : PANDA;
}

/** The robot a URL query string names, else the Panda. */
export function robotFromSearch(search = '') {
  return robotById(new URLSearchParams(search).get('robot'));
}
