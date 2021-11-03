# This is a simple cabinet with a drawer (without handle.

body shelf { X=<T t(0 1.1 0) d(90 0 0 1)> fixed }

shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0  .50 1.)> size=[.8 .03 1.85 .02] contact }
shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0 -.50 1.)> size=[.8 .03 1.85 .02] contact }


shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0 0 1.)> size=[.8 1. .03 .02] contact }
shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0 0 1.3)> size=[.8 1. .03 .02] contact }
shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0 0 1.6)> size=[.8 1. .03 .02] contact }
shape(shelf){ type=ssBox color=[.6 .33 .1] rel=<T t(0 0 1.9)> size=[.8 1. .03 .02] contact }

# shape target(shelf){ type=1 color=[1. 0. 0.] rel=<T t(0 0 1.25)> size=[0 0 0 .07] }


