import pyrealsense2 as rs
import numpy as np
import cv2
import time
import datetime
import os




def main():
    # 創建影像存儲資料夾
    save_dir = "realsense_captures"
    os.makedirs(save_dir, exist_ok=True)

    # 初始化 RealSense 相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)

    # 啟動相機
    pipeline.start(config)
    print("RealSense D435i 已啟動，開始拍照... (每3秒自動拍照)")

    try:
        while True:
            # 獲取影像幀
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue

            # 轉換為 NumPy 陣列
            color_image = np.asanyarray(color_frame.get_data())

            # 產生時間戳記並存檔
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(save_dir, f"capture_{timestamp}.png")
            cv2.imwrite(filename, color_image)

            print(f"拍攝成功: {filename}")

            # 等待 3 秒
            time.sleep(2)

    except KeyboardInterrupt:
        print("\n手動停止，關閉相機...")

    finally:
        pipeline.stop()
        print("RealSense D435i 已關閉。")


if __name__ == '__main__':
    main()
