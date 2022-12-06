import cv2

def make_video(name, frames, fps=30):
    width = frames[0].shape[1]
    height = frames[0].shape[0]
    # Create a VideoWriter object
    video = cv2.VideoWriter(name, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    # Loop through the list of numpy arrays and write each frame to the video
    for frame in frames:
        video.write(frame)

    # Release the VideoWriter object
    video.release()
