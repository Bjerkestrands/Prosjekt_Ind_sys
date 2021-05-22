import cv2 as cv
import numpy as np

def aruco(img, object_corners=[0, 1, 2, 3]):
    all_markers_active = False;

    # Load AruCo markers, in this case we are using the 50 first 6x6 AruCo Markers.
    aruco_dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_50)
    aruco_parameters = cv.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv.aruco.detectMarkers(image, aruco_dictionary, parameters=aruco_parameters)

    # Check if any markers have been detected. If not, return unaltered image.
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):

            # Mark the detected markerID's in object corner  as active.
            if markerID in object_corners:
                marker_active[markerID] = True;

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            coordinates[markerID] = (cX, cY)
            marker_corners[markerID] = (topLeft, topRight, bottomLeft, bottomRight)

        # If all corners are detected and warp is enabled, warp the frame and marker coordinates.
        if all(marker_active):
            all_markers_active = True;

            marker_active[object_corners[0]] = False;
            marker_active[object_corners[1]] = False;
            marker_active[object_corners[2]] = False;
            marker_active[object_corners[3]] = False;

            # Dimensions of the frame.
            dimensions = (img.shape[1], img.shape[0])

            # Declare
            pts1 = np.float32(
                [marker_corners[object_corners[0]][0], marker_corners[object_corners[1]][1],
                 marker_corners[object_corners[2]][2], marker_corners[object_corners[3]][3]])
            pts2 = np.float32([[0, 0], [dimensions[0], 0], [0, dimensions[1]], [dimensions[0], dimensions[1]]])

            # Calculate the perspective transform.
            matrix = cv.getPerspectiveTransform(pts1, pts2)

            return True, matrix
    return False, None

  
def matrix_transform(matrix, input, type):
  # if the input is an image, warp image with cv2
  if type == "img":
      # Dimensions of the frame.
      dimensions = (input.shape[1], input.shape[0])
      # Warp image with cv2
      img = cv.warpPerspective(input, matrix, dimensions)
      return img
  # if the input is a list of coordinates, calculate new coordinates
  elif type == "coordinates":
      for markerID in range(0, len(coordinates)):
          coordinate3D = matrix @ [input[0],
                                   input[1], 1]

          coordinate2D = (int(coordinate3D[0] /
                              coordinate3D[2]),
                          int(coordinate3D[1] /
                              coordinate3D[2]))

      return coordinate2D
    
if __name__ = "__main__":
      console.log("this is a library, run the main file instead")
