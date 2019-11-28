import onnxruntime
import cv2
import numpy as np
from itertools import repeat

from util import red_mask, green_mask, crop_to_bb, find_boxes_to_track


class PredictServer:
    def __init__(self, model_file):
        self.session = onnxruntime.InferenceSession(model_file)
        self.id_to_classname = ['circle', 'square', 'triangle']

    def predict(self, image, *, show_mask=False):
        image = np.asarray(image)
        image = (image - np.mean(image)) / np.std(image)
        image = np.clip(image, -2, 2)
        image = (image * 60 + 120).astype('uint8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_boxes, red_labels = self._predict_red(hsv, show_mask=show_mask)
        green_boxes, green_labels = self._predict_green(hsv, show_mask=show_mask)
        boxes = red_boxes + green_boxes
        labels = red_labels + green_labels
        colours = list(repeat('red', len(red_labels))) + list(repeat('green', len(green_labels)))
        return boxes, labels, colours

    def _predict_red(self, hsv, *, show_mask=False):
        mask = red_mask(hsv)
        if show_mask:
            cv2.imshow('red_mask', mask.astype('float32'))
            cv2.waitKey(1)
        return self._predict_mask(mask)

    def _predict_green(self, hsv, *, show_mask=False):
        mask = green_mask(hsv)
        if show_mask:
            cv2.imshow('green_mask', mask.astype('float32'))
            cv2.waitKey(1)
        return self._predict_mask(mask)

    def _predict_mask(self, mask):
        image = mask.astype('float32')
        image = cv2.erode(image, np.ones((3, 3)))
        image = cv2.dilate(image, np.ones((4, 4)))

        boxes = find_boxes_to_track(image)
        box_labels = []
        for i, (x, y, w, h) in enumerate(boxes):
            box_image = crop_to_bb(image, (x, y, w, h))
            box_image = cv2.resize(box_image, (28, 28))
            box_image = np.expand_dims(box_image, 0)
            box_image = np.tile(box_image, (3, 1, 1))
            pred = self.session.run(None, {"image": np.expand_dims(box_image, 0)})[0]
            box_labels.append(self.id_to_classname[int(np.argmax(pred))])
        return boxes, box_labels


def main():
    server = PredictServer('fastai_trained.onnx')
    cap = cv2.VideoCapture(0)

    colours_by_name = {'red': (255, 0, 0), 'green': (0, 255, 0)}

    while True:
        ret, frame = cap.read()

        boxes, labels, colours = server.predict(frame)
        for (x, y, w, h), label, c in zip(boxes, labels, colours):
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0))
            cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colours_by_name[c])

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()



