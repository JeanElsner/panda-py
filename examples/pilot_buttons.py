"""
This example connects to the Desk to listen for pilot button events.
"""
import sys
import time
import panda_py


def on_event(event):
  """
  Prints the received event. The event is a dictionary
  where the triggering button is the key. The value is either
  True or False for down and up events respectively.
  Refer to the `panda_py.Desk.listen` documentation for mor details.
  """
  print(event)


if __name__ == '__main__':
  if len(sys.argv) < 4:
    raise RuntimeError(
        f'Usage: python {sys.argv[0]} <robot-hostname> <desk-username> <desk-password>'
    )
  # Connect to the Desk
  d = panda_py.Desk(sys.argv[1], sys.argv[2], sys.argv[3])
  # Listen for 30 seconds
  d.listen(on_event)
  time.sleep(30)
  # Stop listening
  d.stop_listen()
