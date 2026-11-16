"""Ghi video offscreen (MUJOCO_GL=egl)."""
import imageio.v2 as imageio
import mujoco as mj


class Recorder:
    def __init__(self, path, model=None, camera=None, size=(640, 480), fps=30, lookat=(0.0, 0.0, 0.75),
                 distance=3.3, azimuth=135.0, elevation=-42.0, speed: float = 1.0):
        self.path, self.camera, self.size, self.fps = path, camera, size, fps
        self.sample_fps = fps / speed   # lấy khung hình theo thời gian mô phỏng; phát ở fps → nhanh gấp `speed` lần
        self.view = (lookat, distance, azimuth, elevation)
        self.renderer = None
        self.writer = imageio.get_writer(path, fps=fps)
        self.cam = None

    def capture(self, d):
        m = d.model if hasattr(d, "model") else None
        if self.renderer is None:
            self.m = m
            self.renderer = mj.Renderer(m, self.size[1], self.size[0])
            self.cam = mj.MjvCamera()
            lookat, dist, az, el = self.view
            self.cam.lookat[:] = lookat
            self.cam.distance, self.cam.azimuth, self.cam.elevation = dist, az, el
        self.renderer.update_scene(d, self.cam if self.camera is None else self.camera)
        self.writer.append_data(self.renderer.render())

    def close(self):
        self.writer.close()
        if self.renderer is not None:
            self.renderer.close()


class LiveViewer:
    """Xem trực tiếp bằng cửa sổ MuJoCo (mujoco.viewer), cùng giao diện với Recorder để CellSim gọi capture().

    Chạy theo thời gian thực nhân `speed`. Trong lúc lập kế hoạch, mô phỏng tạm dừng nên hình đứng yên. Đóng cửa sổ thì
    dừng cảnh (KeyboardInterrupt)."""

    def __init__(self, speed: float = 1.0, fps: int = 60, lookat=(0.0, 0.0, 0.75), distance=3.3, azimuth=135.0,
                 elevation=-42.0):
        self.speed, self.fps = speed, fps
        self.view = (lookat, distance, azimuth, elevation)
        self.viewer = None
        self._t0 = None

    def capture(self, d):
        import time

        import mujoco.viewer
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(d.model, d, show_left_ui=False, show_right_ui=False)
            lookat, dist, az, el = self.view
            self.viewer.cam.lookat[:] = lookat
            self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = dist, az, el
            self._t0 = (time.perf_counter(), d.time)
        if not self.viewer.is_running():
            raise KeyboardInterrupt("đã đóng cửa sổ")
        wall0, sim0 = self._t0
        ahead = (d.time - sim0) / self.speed - (time.perf_counter() - wall0)
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -0.5:  # chậm hơn thời gian thực (ví dụ vừa lập kế hoạch): bắt nhịp lại, không tua nhanh
            self._t0 = (time.perf_counter(), d.time)
        self.viewer.sync()

    def close(self):
        """Đóng cửa sổ và chờ luồng vẽ của viewer dừng hẳn. Thoát tiến trình ngay sau close() (hoặc không close) làm
        cửa sổ treo ("not responding") rồi segfault lúc thoát."""
        import time
        if self.viewer is None:
            return
        self.viewer.close()
        t = time.perf_counter()
        while self.viewer.is_running() and time.perf_counter() - t < 3.0:
            time.sleep(0.05)
        time.sleep(0.3)
        self.viewer = None
