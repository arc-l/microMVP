"""
Main Canvas component for MicroMVP GUI (Pixel-Basis Architecture).

Refactored Logic:
- Scene Coordinate System = Pixel Coordinate System (Top-Left 0,0, Y-Down).
- Transformation: Explicitly calculated in Python (Workspace Meters -> Screen Pixels).
- Rendering: Items are placed at calculated pixel coordinates.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import math

from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QRectF
from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPixmap, QTransform
from PyQt6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsObject,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QSizePolicy,
)

from micromvp.core.models import CarState, WorkspaceConfig

Point = Tuple[float, float]

def load_car_pixmap() -> QPixmap:
    """
    Load car image from package resources.
    """
    import importlib.resources as pkg_resources
    # 注意：这里假设路径结构未变
    try:
        car_png = pkg_resources.files("micromvp.assets") / "carImage.png"
        with pkg_resources.as_file(car_png) as p:
            pm = QPixmap(str(p))
    except ImportError:
        # Fallback for older python versions or different envs if needed
        return QPixmap()
        
    if pm.isNull():
        raise RuntimeError("Failed to load pixmap from micromvp/assets/carImage.png")
    return pm


class CarGraphicsItem(QGraphicsObject):
    """
    Pixel-basis car item with MANUAL hit shape/bounds.

    - Local coordinates: pixels, (0,0) at car center.
    - shape() / boundingRect(): strictly the collision box in pixels.
    - Visuals are child items (pixmap/rect/label), but DO NOT affect hit-test.
    """

    IMAGE_HEADING_OFFSET_DEG = 90.0

    def __init__(
        self,
        car_id: int,
        car_width_m: float,
        car_height_m: float,
        offset_w_m: float,
        offset_h_m: float,
        car_pixmap: QPixmap,
        parent: Optional[QGraphicsItem] = None,
        show_collision_box: bool = True,
        marker_to_axle_offset: Tuple[float, float] = (0.0, 0.0),
    ):
        super().__init__(parent)
        self.car_id = car_id

        # logical (meters)
        self._car_width_m = float(car_width_m)
        self._car_height_m = float(car_height_m)
        self._offset_w_m = float(offset_w_m)
        self._offset_h_m = float(offset_h_m)
        self._marker_dx_m = float(marker_to_axle_offset[0])
        self._marker_dy_m = float(marker_to_axle_offset[1])

        # pixel (updated in update_visuals)
        self._w_px = 1.0
        self._h_px = 1.0
        self._off_w_px = 0.0
        self._off_h_px = 0.0

        # manual hit rect/path (pixel-local)
        self._hit_rect = QRectF(-0.5, -0.5, 1.0, 1.0)
        self._hit_path = QPainterPath()
        self._hit_path.addRect(self._hit_rect)

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)

        # 1) pixmap (child)
        self._pixmap_source = car_pixmap
        self._pixmap_item = QGraphicsPixmapItem(car_pixmap, self)
        self._pixmap_item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        # IMPORTANT: use offset for centering (stable bounds), NOT setPos with scaled size
        pm_w = max(1, car_pixmap.width())
        pm_h = max(1, car_pixmap.height())
        self._pixmap_item.setOffset(-pm_w / 2.0, -pm_h / 2.0)

        # 2) collision rect visual (child)
        self._collision_rect_item: Optional[QGraphicsRectItem] = None
        if show_collision_box:
            self._collision_rect_item = QGraphicsRectItem(self)
            self._collision_rect_item.setBrush(QBrush(Qt.GlobalColor.transparent))
            self._collision_rect_item.setPen(QPen(QColor(30, 60, 90), 2))
            self._collision_rect_item.setZValue(10)

        # 3) label (child)
        self._label = QGraphicsSimpleTextItem(str(car_id), self)
        self._label.setBrush(QBrush(Qt.GlobalColor.white))
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        self._label.setFont(font)
        self._label.setZValue(20)

        # Make sure label never steals clicks
        self._label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self._pixmap_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        if self._collision_rect_item:
            self._collision_rect_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    # QGraphicsObject requires paint(), but we draw with children -> empty paint
    def paint(self, painter, option, widget=None) -> None:
        return

    def boundingRect(self) -> QRectF:
        # scene uses this for indexing; keep it exactly the hit rect (fast + correct)
        return self._hit_rect

    def shape(self) -> QPainterPath:
        # THIS defines click/selection region
        return self._hit_path

    def update_visuals(self, pixels_per_meter: float) -> None:
        # compute pixel dimensions
        w_px = self._car_width_m * pixels_per_meter
        h_px = self._car_height_m * pixels_per_meter
        off_w_px = self._offset_w_m * pixels_per_meter
        off_h_px = self._offset_h_m * pixels_per_meter

        # avoid degenerate sizes
        w_px = max(1.0, float(w_px))
        h_px = max(1.0, float(h_px))

        self.prepareGeometryChange()

        self._w_px = w_px
        self._h_px = h_px
        self._off_w_px = float(off_w_px)
        self._off_h_px = float(off_h_px)

        # update hit rect/path in LOCAL PIXELS
        # Local coords are screen-pixel local (X right, Y down), and the item
        # origin is the *wheel-axle center* (workspace x,y).
        #
        # WorkspaceConfig.offset_w/offset_h are defined in the car body frame
        # when the car faces +Y (up): axle position relative to body bottom-left.
        # For a +Y facing car, the body's bottom is +Y in screen (down), and the
        # body's top is -Y in screen (up). Therefore:
        # - left  = -offset_w
        # - top   = -(car_height - offset_h)
        self._hit_rect = QRectF(
            -self._off_w_px,
            -(self._h_px - self._off_h_px),
            self._w_px,
            self._h_px,
        )
        self._hit_path = QPainterPath()
        self._hit_path.addRect(self._hit_rect)

        # update pixmap scale (keep centered by offset)
        pm_w = max(1, self._pixmap_source.width())
        img_scale = self._w_px / float(pm_w)
        self._pixmap_item.setTransform(QTransform.fromScale(img_scale, img_scale), False)

        # Centre the sprite on the *marker*, not on the body. The sprite is a
        # stand-in — real cars do not look like it — so what makes it useful is
        # lining up with the one thing a person can also see in the camera
        # image. The item origin stays at the axle, which is where the pose is
        # reported and where the selection circle is drawn.
        #
        # marker_to_axle_offset points marker -> axle in the car frame with the
        # car facing +Y, so axle -> marker is its negation. Converting car frame
        # (Y forward) to item-local pixels (Y down) flips the Y sign again,
        # leaving -dx to the right and +dy downward.
        self._pixmap_item.setPos(
            -self._marker_dx_m * pixels_per_meter,
            self._marker_dy_m * pixels_per_meter,
        )
        self._pixmap_item.setZValue(0)

        # update collision rect visual
        if self._collision_rect_item:
            self._collision_rect_item.setRect(self._hit_rect)

        # center label at (0,0)
        b = self._label.boundingRect()
        self._label.setPos(-b.width() / 2.0, -b.height() / 2.0)

    def set_pose_pixels(self, px: float, py: float, theta_deg_ws: float) -> None:
        self.setPos(px, py)
        screen_rot = -theta_deg_ws
        self.setRotation(screen_rot + self.IMAGE_HEADING_OFFSET_DEG)



class MVPCanvas(QGraphicsView):
    """
    Main canvas for MicroMVP.
    
    Architecture:
    - Maintains a logic-to-pixel transform.
    - Stores logical state (_last_car_states, _drawing_data_cache) to re-render on resize.
    """

    canvas_clicked = pyqtSignal(float, float)
    car_clicked = pyqtSignal(int)
    curve_drawn = pyqtSignal(list)

    def __init__(self, workspace_config: WorkspaceConfig, parent=None):
        super().__init__(parent)
        self._ws_config = workspace_config

        # Scene setup (Pixel Coordinates)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        # View settings
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        # We handle scaling manually, so scrollbars are usually off unless zoomed in (here we auto-fit)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setBackgroundBrush(QBrush(QColor(245, 245, 245)))
        
        # Remove default frame
        self.setFrameShape(QGraphicsView.Shape.NoFrame)

        # Resources
        self._car_pixmap = load_car_pixmap()

        # State Storage (Logical)
        self._car_items: Dict[int, CarGraphicsItem] = {}
        self._drawing_items_cache: Dict[str, QGraphicsItem] = {}
        
        # We MUST store the raw data to re-compute pixel positions during resize
        self._last_car_states: Dict[int, CarState] = {}
        self._last_drawings_data: Dict[str, Dict[str, Any]] = {}

        # Transformation Parameters
        self._pixels_per_meter = 10.0  # Scale
        self._offset_x = 0.0           # Margin Left
        self._offset_y = 0.0           # Margin Top (for Y-flip handling)
        self._view_h = 100.0           # Cache view height for Y-flip math

        # Interaction State
        self._draw_curve_enabled = False
        self._click_canvas_enabled = False
        self._is_drawing = False
        self._current_path_ws: List[Point] = [] # Store path in workspace units
        self._temp_path_item: Optional[QGraphicsPathItem] = None
        self._click_point_item: Optional[QGraphicsEllipseItem] = None
        self._boundary_rect_item: Optional[QGraphicsRectItem] = None

    # -------------------------------------------------------------------------
    # Core Coordinate Transformation Logic
    # -------------------------------------------------------------------------

    def _update_transform_params(self):
        """
        Calculate scale and offsets to fit the workspace centrally in the view.
        Called on Resize.
        """
        view_rect = self.viewport().rect()
        vw, vh = view_rect.width(), view_rect.height()
        
        # Workspace Dimensions
        ws_w = self._ws_config.width
        ws_h = self._ws_config.height
        
        if ws_w == 0 or ws_h == 0 or vw == 0 or vh == 0:
            return

        # 1. Calculate Scale (Pixels per Meter) - Fit with padding
        padding_ratio = 0.95
        scale_x = (vw * padding_ratio) / ws_w
        scale_y = (vh * padding_ratio) / ws_h
        self._pixels_per_meter = min(scale_x, scale_y)
        
        # 2. Calculate Centering Offsets
        content_w_px = ws_w * self._pixels_per_meter
        content_h_px = ws_h * self._pixels_per_meter
        
        self._offset_x = (vw - content_w_px) / 2.0
        self._offset_y = (vh - content_h_px) / 2.0
        self._view_h = float(vh)
        
        # Set Scene Rect to match Viewport exactly (Pixel Basis)
        self._scene.setSceneRect(0, 0, vw, vh)

    def workspace_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        """
        Convert Workspace (Meters, Y-Up) -> Screen (Pixels, Y-Down).
        """
        # X: Simple scale + offset
        px = (wx * self._pixels_per_meter) + self._offset_x
        
        # Y: Flip axis.
        # Workspace 0 is at bottom. Screen 0 is at top.
        # logical_y_from_bottom * scale
        # screen_y = (ViewHeight - BottomMargin) - (wy * scale)
        # But simpler: map logical 0 to (ViewHeight - offset_y)
        # content_h = ws_h * scale. Bottom of content is at offset_y + content_h? 
        # Actually standard mapping: 
        # py = ViewHeight - MarginBottom - (wy * scale)
        # MarginBottom is roughly same as MarginTop (_offset_y) if centered.
        # Let's derive from Top:
        # Top of content in screen = _offset_y
        # Top of content in WS = ws_height
        # So: py = _offset_y + (ws_height - wy) * scale
        py = self._offset_y + (self._ws_config.height - wy) * self._pixels_per_meter
        return (px, py)

    def pixel_to_workspace(self, px: float, py: float) -> Tuple[float, float]:
        """
        Convert Screen (Pixels, Y-Down) -> Workspace (Meters, Y-Up).
        """
        if self._pixels_per_meter <= 0: return (0, 0)
        
        wx = (px - self._offset_x) / self._pixels_per_meter
        # Inverse of: py = _offset_y + (ws_h - wy) * scale
        # py - _offset_y = (ws_h - wy) * scale
        # (py - _offset_y)/scale = ws_h - wy
        # wy = ws_h - (py - _offset_y)/scale
        wy = self._ws_config.height - (py - self._offset_y) / self._pixels_per_meter
        return (wx, wy)

    # -------------------------------------------------------------------------
    # Event Handling & Rendering Loop
    # -------------------------------------------------------------------------

    def resizeEvent(self, event):
        """Handle resize: Re-calculate transform and re-position all items."""
        super().resizeEvent(event)
        self._update_transform_params()
        self._redraw_scene()

    def _redraw_scene(self):
        """Force update of all item positions/shapes based on new transform."""
        if self._pixels_per_meter <= 0: return

        # 1. Update Boundary
        self._draw_workspace_boundary()

        # 2. Update Cars
        # We need to re-apply the logical state to the items
        for car_id, state in self._last_car_states.items():
            if car_id in self._car_items:
                item = self._car_items[car_id]
                # Update size/scale
                item.update_visuals(self._pixels_per_meter)
                # Update position
                px, py = self.workspace_to_pixel(state.x, state.y)
                item.set_pose_pixels(px, py, state.theta)

        # 3. Update Drawings
        for uuid, data in self._last_drawings_data.items():
            if uuid in self._drawing_items_cache:
                self._update_drawing_geometry(self._drawing_items_cache[uuid], data)

        # 4. Update Click Point (if active)
        if self._click_point_item and self._click_point_item.isVisible():
             # We assume click point is transient, but if we wanted to persist it during resize,
             # we would need its logical position. For now, let's just hide it or leave it.
             # Better: clear it on resize to avoid confusion.
             self._clear_click_point()

    def _draw_workspace_boundary(self):
        """Draw the dashed box representing the workspace boundaries."""
        if not self._boundary_rect_item:
            self._boundary_rect_item = QGraphicsRectItem()
            pen = QPen(QColor(100, 100, 100), 2, Qt.PenStyle.DashLine)
            self._boundary_rect_item.setPen(pen)
            self._boundary_rect_item.setBrush(Qt.GlobalColor.transparent)
            self._scene.addItem(self._boundary_rect_item)
        
        # Calculate pixel rect for workspace (0,0) to (w,h)
        tl_x, tl_y = self.workspace_to_pixel(0, self._ws_config.height) # Top-Left in screen
        br_x, br_y = self.workspace_to_pixel(self._ws_config.width, 0)  # Bottom-Right in screen
        
        self._boundary_rect_item.setRect(tl_x, tl_y, br_x - tl_x, br_y - tl_y)

    # -------------------------------------------------------------------------
    # Public API: Car Updates
    # -------------------------------------------------------------------------

    def update_cars(self, car_states: Dict[int, CarState]) -> None:
        """Update car positions."""
        self._last_car_states = car_states # Cache logical state
        
        current_ids = set(car_states.keys())
        existing_ids = set(self._car_items.keys())

        # Remove dead cars
        for cid in existing_ids - current_ids:
            self._scene.removeItem(self._car_items.pop(cid))

        # Add/Update cars
        for cid, state in car_states.items():
            if cid not in self._car_items:
                item = CarGraphicsItem(
                    car_id=cid,
                    car_width_m=self._ws_config.car_width,
                    car_height_m=self._ws_config.car_height,
                    offset_w_m=self._ws_config.offset_w,
                    offset_h_m=self._ws_config.offset_h,
                    car_pixmap=self._car_pixmap,
                    marker_to_axle_offset=self._ws_config.marker_to_axle_offset,
                )
                self._scene.addItem(item)
                self._car_items[cid] = item
            
            # Update visuals (scale) and pose
            item = self._car_items[cid]
            item.update_visuals(self._pixels_per_meter)
            
            px, py = self.workspace_to_pixel(state.x, state.y)
            item.set_pose_pixels(px, py, state.theta)

    # -------------------------------------------------------------------------
    # Public API: Drawings
    # -------------------------------------------------------------------------

    def update_drawings(self, drawings: List[Dict[str, Any]]) -> None:
        """
        Update retained-mode drawings.
        """
        current_uuids = set()

        for drawing in drawings:
            uuid = drawing.get("uuid")
            if not uuid: continue
            
            current_uuids.add(uuid)
            self._last_drawings_data[uuid] = drawing # Cache logical data

            if uuid not in self._drawing_items_cache:
                item = self._create_drawing_item(drawing)
                if item:
                    self._scene.addItem(item)
                    self._drawing_items_cache[uuid] = item
            
            # Update geometry based on new data and current scale
            if uuid in self._drawing_items_cache:
                self._update_drawing_geometry(self._drawing_items_cache[uuid], drawing)

        # Cleanup
        stale_uuids = set(self._drawing_items_cache.keys()) - current_uuids
        for uuid in stale_uuids:
            self._scene.removeItem(self._drawing_items_cache.pop(uuid))
            if uuid in self._last_drawings_data:
                del self._last_drawings_data[uuid]

    def _create_drawing_item(self, drawing: Dict[str, Any]) -> Optional[QGraphicsItem]:
        dtype = drawing.get("type", "")
        if dtype == "line": return QGraphicsLineItem()
        if dtype == "circle" or dtype == "point": return QGraphicsEllipseItem()
        if dtype == "rect": return QGraphicsRectItem()
        if dtype == "path": return QGraphicsPathItem()
        return None

    def _update_drawing_geometry(self, item: QGraphicsItem, drawing: Dict[str, Any]):
        """Convert logical drawing data to pixel geometry."""
        dtype = drawing.get("type", "")
        scale = self._pixels_per_meter

        # Apply Style
        color = QColor(drawing.get("color", "#FF0000"))
        width = drawing.get("width", 2) # This is now Pixel width
        pen = QPen(color, width)
        # In pixel-basis, cosmetic is not strictly needed if we want true pixel width,
        # but cosmetic is safer for performance on some Qt backends. 
        # Here we leave it default (False) so width=2 means 2 pixels.
        item.setPen(pen)
        
        if "fill" in drawing and hasattr(item, "setBrush"):
            item.setBrush(QBrush(QColor(drawing["fill"])))
        elif hasattr(item, "setBrush"):
            item.setBrush(Qt.GlobalColor.transparent)

        # Apply Geometry
        if dtype == "line" and isinstance(item, QGraphicsLineItem):
            x1, y1 = drawing.get("start", (0, 0))
            x2, y2 = drawing.get("end", (0, 0))
            px1, py1 = self.workspace_to_pixel(x1, y1)
            px2, py2 = self.workspace_to_pixel(x2, y2)
            item.setLine(px1, py1, px2, py2)

        elif dtype == "circle" and isinstance(item, QGraphicsEllipseItem):
            cx, cy = drawing.get("center", (0, 0))
            r_m = drawing.get("radius", 0.1) # Radius in meters
            px, py = self.workspace_to_pixel(cx, cy)
            r_px = r_m * scale
            item.setRect(px - r_px, py - r_px, r_px * 2, r_px * 2)

        elif dtype == "point" and isinstance(item, QGraphicsEllipseItem):
            # Points are usually constant size in Pixels
            cx, cy = drawing.get("position", (0, 0))
            r_px = drawing.get("radius", 4) # Radius in Pixels
            px, py = self.workspace_to_pixel(cx, cy)
            item.setRect(px - r_px, py - r_px, r_px * 2, r_px * 2)

        elif dtype == "rect" and isinstance(item, QGraphicsRectItem):
            x, y = drawing.get("position", (0, 0)) # Usually bottom-left or top-left in WS?
            w, h = drawing.get("size", (0.1, 0.1))
            # Assume Rect position is WS Top-Left or Bottom-Left? 
            # Let's assume standard math: (x,y) is min_x, min_y (Bottom-Left).
            # We need to convert (x, y+h) [Top-Left WS] to screen
            px_l, py_t = self.workspace_to_pixel(x, y + h)
            w_px = w * scale
            h_px = h * scale
            item.setRect(px_l, py_t, w_px, h_px)
            
        elif dtype == "path" and isinstance(item, QGraphicsPathItem):
            points = drawing.get("points", [])
            path = QPainterPath()
            if points:
                px0, py0 = self.workspace_to_pixel(points[0][0], points[0][1])
                path.moveTo(px0, py0)
                for pt in points[1:]:
                    px, py = self.workspace_to_pixel(pt[0], pt[1])
                    path.lineTo(px, py)
            item.setPath(path)

    # -------------------------------------------------------------------------
    # Interaction
    # -------------------------------------------------------------------------

    def enable_curve_drawing(self, enabled: bool) -> None:
        self._draw_curve_enabled = enabled

    def enable_canvas_click(self, enabled: bool) -> None:
        self._click_canvas_enabled = enabled

    def _show_click_point(self, wx: float, wy: float):
        if not self._click_point_item:
            self._click_point_item = QGraphicsEllipseItem()
            self._click_point_item.setPen(QPen(Qt.GlobalColor.red, 2))
            self._click_point_item.setBrush(QBrush(Qt.GlobalColor.red))
            self._scene.addItem(self._click_point_item)
        
        px, py = self.workspace_to_pixel(wx, wy)
        r = 3.0 # Pixels
        self._click_point_item.setRect(px - r, py - r, r*2, r*2)
        self._click_point_item.show()

    def _clear_click_point(self):
        if self._click_point_item:
            self._click_point_item.hide()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            # 1. Handle Car Clicks
            # Scene pos is Pixel pos
            items = self.items(event.pos())
            for it in items:
                # Traverse up to find CarGraphicsItem
                curr = it
                while curr:
                    if isinstance(curr, CarGraphicsItem):
                        self.car_clicked.emit(curr.car_id)
                        return # Car click consumes event
                    curr = curr.parentItem()

            # 2. Handle Canvas Clicks / Drawing
            scene_pos = self.mapToScene(event.pos())
            px, py = scene_pos.x(), scene_pos.y()
            wx, wy = self.pixel_to_workspace(px, py)

            if self._draw_curve_enabled:
                self._is_drawing = True
                self._current_path_ws = [(wx, wy)]
                
                # Setup visual path
                self._temp_path_item = QGraphicsPathItem()
                self._temp_path_item.setPen(QPen(QColor(255, 100, 100), 2))
                self._scene.addItem(self._temp_path_item)
                
                # Feedback point
                if self._click_canvas_enabled:
                    self._show_click_point(wx, wy)

            elif self._click_canvas_enabled:
                self._show_click_point(wx, wy)
                self.canvas_clicked.emit(wx, wy)

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._is_drawing and self._temp_path_item:
            scene_pos = self.mapToScene(event.pos())
            px, py = scene_pos.x(), scene_pos.y()
            wx, wy = self.pixel_to_workspace(px, py)
            
            self._current_path_ws.append((wx, wy))

            # Rebuild path in pixels
            path = QPainterPath()
            start_px, start_py = self.workspace_to_pixel(self._current_path_ws[0][0], self._current_path_ws[0][1])
            path.moveTo(start_px, start_py)
            
            # Optimization: don't convert ALL points every move if list is huge, 
            # but for GUI interaction it's usually fine.
            for ptw in self._current_path_ws[1:]:
                ix, iy = self.workspace_to_pixel(ptw[0], ptw[1])
                path.lineTo(ix, iy)
            
            self._temp_path_item.setPath(path)
            
            if self._click_canvas_enabled and len(self._current_path_ws) > 5:
                self._clear_click_point()

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._is_drawing:
            self._is_drawing = False
            
            # Emit Curve
            if len(self._current_path_ws) > 5:
                self.curve_drawn.emit(self._current_path_ws)
            elif self._click_canvas_enabled and len(self._current_path_ws) > 0:
                # Treated as a click
                p = self._current_path_ws[0]
                self.canvas_clicked.emit(p[0], p[1])
            
            # Cleanup
            if self._temp_path_item:
                self._scene.removeItem(self._temp_path_item)
                self._temp_path_item = None
            self._current_path_ws = []
            self._clear_click_point()

        elif event.button() == Qt.MouseButton.LeftButton:
            if self._click_canvas_enabled:
                self._clear_click_point()

        super().mouseReleaseEvent(event)

    # Legacy method compatibility (optional, if you need view_to_workspace elsewhere)
    def view_to_workspace(self, vx: float, vy: float) -> Tuple[float, float]:
        scene_p = self.mapToScene(int(vx), int(vy))
        return self.pixel_to_workspace(scene_p.x(), scene_p.y())
