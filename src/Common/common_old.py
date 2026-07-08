'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
#!/usr/bin/env python
from PyQt5.QtCore import QMutex, Qt, pyqtSignal, QPointF
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsTextItem
from PyQt5.QtGui import QBrush, QPen, QColor, QFont, QWheelEvent

import ROS_Node.ros_common as ros_common
import numpy as np

class CommonData(): # store the data from the ROS nodes
    def __init__(self):
        self.msg = ""
        self.current_Time = ""
        
        self.current_distance = ""
        self.total_distance = 0

        self.current_imu = ros_common.IMUinfo()
        self.current_global_pos = ros_common.GlobalPositionInfo()
        self.current_local_pos = ros_common.Point3()
        self.target_global_pos = ros_common.GlobalPositionInfo()
        self.target_local_pos = ros_common.Point3()
        self.home_global_pos = ros_common.GlobalPositionInfo()
        self.home_local_pos = ros_common.Point3()

        self.current_state = ros_common.StateInfo()
        self.current_POI_info = ros_common.POI_info()

        self.sampling_time = 0.0

        self.lock = QMutex()

    def update_imu(self, x, y, z, w):
        # NED to ENU conversion
        quat_enu = self.ned_to_enu(x, y, z, w)
        euler = self.quat_to_euler(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3])
    
        if not self.lock.tryLock():
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return
    
    def update_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.current_global_pos.latitude = latitude
        self.current_global_pos.longitude = longitude
        self.current_global_pos.altitude = altitude
        local_x, local_y, local_z = self.global_to_local(latitude, longitude, altitude)
        self.current_local_pos.x = local_x
        self.current_local_pos.y = local_y
        self.current_local_pos.z = local_z
        self.lock.unlock()
        return
    
    def update_target_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.target_global_pos.latitude = latitude
        self.target_global_pos.longitude = longitude
        self.target_global_pos.altitude = altitude
        self.lock.unlock()
        return
    
    def update_target_local_pos(self, local_x, local_y, local_z):
        if not self.lock.tryLock():
            return
        self.target_local_pos.x = local_x
        self.target_local_pos.y = local_y
        self.target_local_pos.z = local_z
        self.lock.unlock()
        return

    def update_home_global_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.home_global_pos.latitude = latitude
        self.home_global_pos.longitude = longitude
        self.home_global_pos.altitude = altitude
        self.lock.unlock()
        return

    def update_bridge_state(self, bridge_state):

        if not self.lock.tryLock():
            return
        self.current_state.bridge_state = bridge_state
        self.lock.unlock()
        return
    
    def update_planner_state(self, planner_state):

        if not self.lock.tryLock():
            return
        self.current_state.planner_state = planner_state
        self.lock.unlock()
        return
    
    def update_drone_state(self, drone_state):

        if not self.lock.tryLock():
            return
        self.current_state.drone_state = drone_state
        self.lock.unlock()
        return

    def update_POI_list(self, POI_list):
        if not self.lock.tryLock():
            return
        self.current_POI_info.POI_list = POI_list
        self.lock.unlock()
        return
    
    def update_visited_sequence(self, visited_sequence):
        if not self.lock.tryLock():
            return
        self.current_POI_info.visited_sequence = visited_sequence
        self.lock.unlock()
        return
    
    def update_best_sequence(self, best_sequence):
        if not self.lock.tryLock():
            return
        self.current_POI_info.best_sequence = best_sequence
        self.lock.unlock()
        return
    
    def update_sampling_time(self, sampling_time):
        if not self.lock.tryLock():
            return
        self.sampling_time = sampling_time
        self.lock.unlock()

    def update_imu(self, x, y, z, w):
        # NED to ENU conversion
        quat_enu = self.ned_to_enu(x, y, z, w)
        euler = self.quat_to_euler(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3])

        if not self.lock.tryLock():
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return
    
    
    def global_to_local(self, lat_deg, lon_deg, alt_m):
        # Local approximation around home
        R = 6378137.0  # WGS84 Earth radius approximation

        dlat_rad = np.radians(lat_deg - self.home_global_pos.latitude)
        dlon_rad = np.radians(lon_deg - self.home_global_pos.longitude)

        local_x = R * np.cos(np.radians(self.home_global_pos.latitude)) * dlon_rad
        local_y = R * dlat_rad
        local_z = alt_m - self.home_global_pos.altitude

        return local_x, local_y, local_z
    
    def quat_to_euler(self, x, y, z, w):
        quat_enu = [x, y, z, w]
        nrm = abs(sum(q**2 for q in quat_enu) - 1.0)
        quat = [0, 0, 0, 1] if nrm > 1e-5 else quat_enu

        # Manual quaternion to Euler conversion (XYZ order)
        # quat is [x, y, z, w] format
        qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        euler = np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

        # convert to 360 coordinates
        if euler[2] < 0:
            euler[2] = euler[2] + 360

        return euler
    
    def ned_to_enu(self, x, y, z, w):
        # Manual quaternion operations without scipy
        q = [w, x, y, z]  # scalar first format

        # ---- Fixed transforms
        # ENU -> NED
        Rie = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # FLU -> FRD
        Rbv = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        # Convert rotation matrices to quaternions
        q_rie = self._rotation_matrix_to_quat(Rie)
        q_rbv = self._rotation_matrix_to_quat(Rbv)

        # Quaternion multiplication: q_rie * q * q_rbv
        q_temp = self._quat_multiply(q_rie, q)
        q_result = self._quat_multiply(q_temp, q_rbv)

        # Return as [x, y, z, w] format (standard format)
        return [q_result[1], q_result[2], q_result[3], q_result[0]]

    def _rotation_matrix_to_quat(self, R):
        """Convert rotation matrix to quaternion [w, x, y, z]"""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return [w, x, y, z]

    def _quat_multiply(self, q1, q2):
        """Multiply two quaternions [w, x, y, z]"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return [w, x, y, z]


class MapView(QGraphicsView):
    poi_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0
        self.scale_factor = 10.0  # pixels/m

        # Setup scene
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.scene.setSceneRect(-5000,-5000,10000,10000)

        # Enable panning and zooming
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.NoDrag)

        self._panning = False
        self._pan_start = QPointF()

        # Zoom limits
        self._zoom_min = 0.1
        self._zoom_max = 20.0
        self._zoom_level = 1.0

        # Marker references for cleanup
        self.home_marker = None
        self.home_label = None
        self.drone_marker = None
        self.poi_markers = []
        self.visited_lines = []
        self.best_seq_lines = []
        self.temp_poi_markers = []

    # ─── Coordinate helpers ───────────────────────────────────────────────────

    def update_map_offset(self, offset_x, offset_y):
        self.map_offset_x = offset_x
        self.map_offset_y = offset_y

    def update_scale_factor(self, scale_factor):
        self.scale_factor = scale_factor

    def map_to_local(self, x, y):
        local_x = (x - self.map_offset_x) / self.scale_factor
        local_y = -(y - self.map_offset_y) / self.scale_factor
        return local_x, local_y

    def local_to_map(self, local_x, local_y):
        x = local_x * self.scale_factor + self.map_offset_x
        y = -(local_y * self.scale_factor + self.map_offset_y)
        return x, y

    # ─── Zoom ─────────────────────────────────────────────────────────────────

    def wheelEvent(self, event: QWheelEvent):
        zoom_in_factor = 1.15
        zoom_out_factor = 1 / zoom_in_factor

        if event.angleDelta().y() > 0:
            factor = zoom_in_factor
        else:
            factor = zoom_out_factor

        # Clamp zoom level
        new_zoom = self._zoom_level * factor
        if new_zoom < self._zoom_min or new_zoom > self._zoom_max:
            return

        self._zoom_level = new_zoom
        self.scale(factor, factor)

    # ─── Pan ──────────────────────────────────────────────────────────────────

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton or event.button() == Qt.RightButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        elif event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            x, y = scene_pos.x(), scene_pos.y()
            self.draw_temp_POI(x, y)
            self.poi_clicked.emit(x, y)
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton or event.button() == Qt.RightButton:
            self._panning = False
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return

        super().mouseReleaseEvent(event)

    # ─── Reset view ───────────────────────────────────────────────────────────

    def reset_view(self):
        self.resetTransform()
        self._zoom_level = 1.0

        # Collect only real items, excluding temp POIs if desired
        rect = self.scene.itemsBoundingRect()
        if rect.isEmpty():
            self.centerOn(0, 0)
            return

        rect = rect.adjusted(-50, -50, 50, 50)
        self.fitInView(rect, Qt.KeepAspectRatio)

    # ─── Cleanup helpers ──────────────────────────────────────────────────────

    def _remove_items(self, item_list):
        for item in item_list:
            if item and item.scene():
                self.scene.removeItem(item)
        item_list.clear()

    def _remove_item(self, item):
        if item and item.scene():
            self.scene.removeItem(item)

    # ─── Draw home ────────────────────────────────────────────────────────────

    def draw_home(self, home_local_x, home_local_y):
        """Home is always the local origin (0, 0)"""
        self._remove_item(self.home_marker)
        self._remove_item(self.home_label)

        map_x, map_y = self.local_to_map(home_local_x, home_local_y)
        size = 14

        self.home_marker = self.scene.addEllipse(
            map_x - size / 2, map_y - size / 2, size, size,
            QPen(QColor("#1a7a1a"), 2),
            QBrush(QColor("#2ecc71"))
        )
        self.home_marker.setZValue(10)

        self.home_label = self.scene.addText("HOME")
        self.home_label.setDefaultTextColor(QColor("#2ecc71"))
        font = QFont(); font.setBold(True); font.setPointSize(8)
        self.home_label.setFont(font)
        self.home_label.setPos(map_x + size, map_y - size)
        self.home_label.setZValue(10)

    # ─── Draw drone ───────────────────────────────────────────────────────────

    def draw_follower_drone(self, local_x, local_y):
        self._remove_item(self.drone_marker)

        map_x, map_y = self.local_to_map(local_x, local_y)
        size = 12

        self.drone_marker = self.scene.addEllipse(
            map_x - size / 2, map_y - size / 2, size, size,
            QPen(QColor("#1a1aff"), 2),
            QBrush(QColor("#3498db"))
        )
        self.drone_marker.setZValue(10)

    # ─── Draw POIs ────────────────────────────────────────────────────────────

    def draw_temp_POI(self, scene_x, scene_y):
        """Add a temporary POI marker at the clicked scene position."""
        size = 10
        pen = QPen(QColor("#95a5a6"), 2, Qt.DashLine)
        brush = QBrush(Qt.NoBrush)

        marker = self.scene.addEllipse(
            scene_x - size / 2, scene_y - size / 2, size, size,
            pen, brush
        )
        marker.setZValue(6)
        self.temp_poi_markers.append(marker)

    def clear_temp_POIs(self):
        """Clear all temporary POI markers."""
        self._remove_items(self.temp_poi_markers)  
    
    def draw_pois(self, poi_list, visited_sequence, best_sequence):
        """
        poi_list        : list of (x, y, z) in local coords, 1-indexed in sequences
        visited_sequence: list of int indices (1-based) already visited
        best_sequence   : list of int indices (1-based) — planned best path
        """
        # Clean up previous drawings
        self._remove_items(self.poi_markers)
        self._remove_items(self.visited_lines)
        self._remove_items(self.best_seq_lines)

        if not poi_list:
            return

        visited_set = set(visited_sequence)  # for O(1) lookup

        # ── 1. Draw visited path (red solid) ──────────────────────────────
        visited_pen = QPen(QColor("#e74c3c"), 2, Qt.SolidLine)

        # Connect home to first visited
        if len(visited_sequence) > 0:
            idx_a = visited_sequence[0]
            if 0 <= idx_a < len(poi_list):
                home_x, home_y = self.local_to_map(0.0, 0.0)
                bx, by = self.local_to_map(poi_list[idx_a][0], poi_list[idx_a][1])
                line = self.scene.addLine(home_x, home_y, bx, by, visited_pen)
                line.setZValue(2)
                self.visited_lines.append(line)

        # Connect visited sequence
        for i in range(len(visited_sequence) - 1):
            idx_a = visited_sequence[i]
            idx_b = visited_sequence[i + 1]
            if 0 <= idx_a < len(poi_list) and 0 <= idx_b < len(poi_list):
                ax, ay = self.local_to_map(poi_list[idx_a][0], poi_list[idx_a][1])
                bx, by = self.local_to_map(poi_list[idx_b][0], poi_list[idx_b][1])
                line = self.scene.addLine(ax, ay, bx, by, visited_pen)
                line.setZValue(2)
                self.visited_lines.append(line)

        # ── 2. Draw best sequence path (orange dashed) ────────────────────
        best_pen = QPen(QColor("#e67e22"), 2, Qt.DashLine)

        # Connect last visited to first best sequence
        if len(visited_sequence) > 0 and len(best_sequence) > 0:
            idx_a = visited_sequence[-1]
            idx_b = best_sequence[0]
            if 0 <= idx_a < len(poi_list) and 0 <= idx_b < len(poi_list):
                ax, ay = self.local_to_map(poi_list[idx_a][0], poi_list[idx_a][1])
                bx, by = self.local_to_map(poi_list[idx_b][0], poi_list[idx_b][1])
                line = self.scene.addLine(ax, ay, bx, by, best_pen)
                line.setZValue(1)
                self.best_seq_lines.append(line)
        elif len(best_sequence) > 0:
            # No visited yet — connect home to first best sequence instead
            idx_b = best_sequence[0]
            if 0 <= idx_b < len(poi_list):
                home_x, home_y = self.local_to_map(0.0, 0.0)
                bx, by = self.local_to_map(poi_list[idx_b][0], poi_list[idx_b][1])
                line = self.scene.addLine(home_x, home_y, bx, by, best_pen)
                line.setZValue(1)
                self.best_seq_lines.append(line)

        # Connect best sequence
        for i in range(len(best_sequence) - 1):
            idx_a = best_sequence[i]
            idx_b = best_sequence[i + 1]
            if 0 <= idx_a < len(poi_list) and 0 <= idx_b < len(poi_list):
                ax, ay = self.local_to_map(poi_list[idx_a][0], poi_list[idx_a][1])
                bx, by = self.local_to_map(poi_list[idx_b][0], poi_list[idx_b][1])
                line = self.scene.addLine(ax, ay, bx, by, best_pen)
                line.setZValue(1)
                self.best_seq_lines.append(line)

        # Connect last best sequence back to home
        if len(best_sequence) > 0:
            idx_a = best_sequence[-1]
            if 0 <= idx_a < len(poi_list):
                ax, ay = self.local_to_map(poi_list[idx_a][0], poi_list[idx_a][1])
                home_x, home_y = self.local_to_map(0.0, 0.0)
                line = self.scene.addLine(ax, ay, home_x, home_y, best_pen)
                line.setZValue(1)
                self.best_seq_lines.append(line)

        # ── 3. Draw POI markers ───────────────────────────────────────────
        for i, (lx, ly, lz) in enumerate(poi_list):
            poi_idx = i
            map_x, map_y = self.local_to_map(lx, ly)
            size = 10

            is_visited = poi_idx in visited_set

            if is_visited:
                # Visited: solid red fill
                pen   = QPen(QColor("#922b21"), 2)
                brush = QBrush(QColor("#e74c3c"))
                label_color = QColor("#e74c3c")
            else:
                # Unvisited: hollow yellow
                pen   = QPen(QColor("#d4ac0d"), 2)
                brush = QBrush(Qt.NoBrush)
                label_color = QColor("#f1c40f")

            marker = self.scene.addEllipse(
                map_x - size / 2, map_y - size / 2, size, size,
                pen, brush
            )
            marker.setZValue(5)
            self.poi_markers.append(marker)

            # Index label next to marker
            label = self.scene.addText(str(poi_idx + 1))
            label.setDefaultTextColor(label_color)
            font = QFont(); font.setPointSize(7)
            label.setFont(font)
            label.setPos(map_x + size, map_y - size / 2)
            label.setZValue(5)
            self.poi_markers.append(label)