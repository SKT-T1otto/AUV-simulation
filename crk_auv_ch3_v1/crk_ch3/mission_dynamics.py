"""G5 source-pinned motion boundary; native step/event order stays unchanged.

_apply_agent_dynamics retains the pinned method verbatim except its vehicle
integration block. Test checks the retained prefix and suffix against the pin.
No source snapshot files are patched or imported from a sibling repository.
"""
import numpy as np
import torch

from .native_source import activate_native
activate_native()
from core.env.task_protocol import STRICT, LEGACY, closed_segment_aabb_first_hit
from .mission_transport import MotionRequest


def exchange_motion(env, old_position, blocked):
    acceleration = env._agent_acc.detach().cpu().numpy().copy()
    acceleration[blocked.detach().cpu().numpy()] = 0.
    request = MotionRequest.create(int(env.step_count)+1, old_position.detach().cpu().numpy(),
                                   env._agent_vel.detach().cpu().numpy(), acceleration,
                                   blocked.detach().cpu().tolist())
    feedback = env._crk_motion_exchange.exchange(request)
    # Raw measured endpoint is retained by the exchange. Native task bookkeeping
    # below can project collisions, clamp bounds or freeze finished agents. These
    # are task-side values and MUST NOT be written back to Gazebo as a teleport.
    env._agent_pos.copy_(env._vec(feedback.positions.copy()))
    env._agent_vel.copy_(env._vec(feedback.velocities_world.copy()))


class ExternalDynamics:
    def _apply_agent_dynamics(self, actions):
        actions = torch.clamp(self._actions_to_tensor(actions), -1.0, 1.0)
        self._collision_flags.zero_()
        old_position = self._agent_pos.clone()
        blocked = self.agent_finished.clone()
        actions = actions.clone()
        actions[blocked] = 0.0
        raw_residual = self._actions_to_residual_acc(actions)
        residual = self._residual_scale[:, None] * raw_residual
        prior = self._prior_strength[:, None] * self._compute_waypoint_prior_acc() if self.use_residual_prior else torch.zeros_like(raw_residual)
        desired_acceleration = prior + residual
        desired_acceleration[:, :2] = torch.clamp(desired_acceleration[:, :2], -self._a_xy_max[:, None], self._a_xy_max[:, None])
        desired_acceleration[:, 2] = torch.clamp(desired_acceleration[:, 2], -self._a_z_max, self._a_z_max)
        self._last_residual_acc.copy_(residual.detach())
        self._last_prior_acc.copy_(prior.detach())
        self.last_residual_norm = float(torch.norm(residual, dim=1).mean())
        self._record_residual_prior_diagnostics(prior, residual, desired_acceleration)
        self._agent_acc.copy_(desired_acceleration)
        # G5 replaces only the reference vehicle integrator, preserving the
        # native action composition and task collision/boundary bookkeeping.
        exchange_motion(self, old_position, blocked)
        if self.task_protocol == STRICT:
            self._agent_pos[blocked] = old_position[blocked]
            candidate = self._agent_pos.clone()
            self.last_collision_records = []
            for agent_id in range(4):
                start = old_position[agent_id].detach().cpu().numpy().astype(np.float64)
                end = candidate[agent_id].detach().cpu().numpy().astype(np.float64)
                hits = []
                for (obstacle_id, obstacle) in enumerate(self.obstacles):
                    (center, half) = (np.asarray(obstacle['center']), np.asarray(obstacle['size']) / 2)
                    tau = closed_segment_aabb_first_hit(start, end, center - half, center + half)
                    if tau is not None:
                        hits.append((tau, obstacle_id))
                if hits:
                    (tau, obstacle_id) = min(hits)
                    point = start + tau * (end - start)
                    self._agent_pos[agent_id] = self._vec(point)
                    self._agent_vel[agent_id] = 0.0
                    self._collision_flags[agent_id] = True
                    self.last_collision_records.append({'agent_id': agent_id, 'obstacle_id': obstacle_id, 'position_before': start.tolist(), 'candidate_position': end.tolist(), 'first_hit_point': point.tolist(), 'hit_fraction': tau})
        clipped = torch.clamp(self._agent_pos, min=self._lower_bound, max=self.space_size)
        wall = torch.any(clipped != self._agent_pos, dim=1)
        self._agent_pos.copy_(clipped)
        self._agent_vel[wall] *= 0.4
        inside = self._points_inside_obstacles(self._agent_pos)
        if self.task_protocol == LEGACY and torch.any(inside):
            self._agent_pos[inside] = old_position[inside]
            self._agent_vel[inside] *= -0.2
            self._collision_flags[inside] = True
        self._agent_pos[blocked] = old_position[blocked]
        self._agent_vel[blocked] = 0.0
        self._agent_acc[blocked] = 0.0
        if not self.task_found:
            self.standby_executor_travel_distance += float(torch.norm(self._agent_pos[self.executor_idx] - old_position[self.executor_idx]).item())
        self.step_count += 1
