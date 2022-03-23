import os
import shutil
import cv2
from copy import copy

template_dir = '../templates'
def label_images():
  label_index = {}
  for d in os.listdir(template_dir):
    if os.path.isdir(os.path.join(template_dir, d)) and d != 'unknown': 
      label_index[d] = 1
      for f in os.listdir(os.path.join(template_dir, d)):
        if os.path.isfile(os.path.join(template_dir, d, f)):
          label_index[d] = max(label_index[d], int(f.split('.')[0].lstrip(d)) + 1)
  if len(label_index):
    print("Your current labels are:")
    for l in label_index.keys():
      print(l)
  print("Enter any labels you want to add: (enter c to continue)")
  label = ''
  while True:
    label = input()
    if label == 'c':
      break
    if not os.path.exists(os.path.join(template_dir, label)):
      os.makedirs(os.path.join(template_dir, label))
      label_index[label] = 1

  for f in os.listdir(os.path.join(template_dir, 'unknown')):
    path = os.path.join(template_dir, 'unknown', f)
    cv2.namedWindow('Template')
    for label in label_index.keys():
      cb = label_callback(path, label, label_index)
      cv2.createButton(label, cb, None, cv2.QT_PUSH_BUTTON, 1)
    cv2.createButton('Delete', lambda *args: delete(path), None, cv2.QT_PUSH_BUTTON, 1)
    image = cv2.imread(path)
    cv2.imshow('Template', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def label_callback(file, label, label_index):
  def callback(*args):
    shutil.move(
        file
        , os.path.join(template_dir, label, '{}{}.jpg'.format(label, label_index[label]))
      )
    label_index[label] += 1
    cv2.destroyAllWindows()
  return callback

def delete(file):
  os.remove(file)
  cv2.destroyAllWindows()

if __name__ == '__main__':
  label_images()
  