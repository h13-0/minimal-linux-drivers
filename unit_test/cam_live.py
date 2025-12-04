import cv2

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('R', 'G', 'B', '3'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print(f"Backend: {cap.getBackendName()}")
    print(f"Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

    # 状态量
    h_flip = False      # 水平翻转
    v_flip = False      # 垂直翻转
    scale = 1.0         # 缩放倍数
    rot_k = 0           # 旋转次数（每次 90° 顺时针，0~3）

    while True:
        ret, frame = cap.read()
        print(f"ret={ret}")
        if not ret:
            input("Press Enter to continue...")
            continue

        print(f"frame.shape={frame.shape}")

        # 先复制一份，避免直接修改原始 frame
        img = frame.copy()

        # 1. 翻转
        if h_flip:
            img = cv2.flip(img, 1)  # 水平翻转
        if v_flip:
            img = cv2.flip(img, 0)  # 垂直翻转

        # 2. 旋转（顺时针，每次 90°）
        if rot_k == 1:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif rot_k == 2:
            img = cv2.rotate(img, cv2.ROTATE_180)
        elif rot_k == 3:
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # 3. 缩放
        if scale != 1.0:
            h, w = img.shape[:2]
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        cv2.imshow("cam", img)

        # 键盘处理
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # q 或 ESC 退出
            break
        elif key == ord('f'):
            # 水平翻转开关
            h_flip = not h_flip
            print("水平翻转:", h_flip)
        elif key == ord('v'):
            # 垂直翻转开关
            v_flip = not v_flip
            print("垂直翻转:", v_flip)
        elif key == ord('+'):
            scale = min(scale + 0.1, 3.0)
            print("缩放倍数:", scale)
        elif key == ord('-'):
            scale = max(scale - 0.1, 0.1)
            print("缩放倍数:", scale)
        elif key == ord('r'):
            rot_k = (rot_k + 1) % 4
            print("旋转次数(90°步进):", rot_k)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
