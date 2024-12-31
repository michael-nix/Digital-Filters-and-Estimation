# Using and Designing Kalman Filters

Without re-deriving the theory of how Kalman filters came to be, we review what they are and how to go about using them.  Walking through three very basic theoretical examples, we show how state estimates are generally calculated from predictions and measurements, opening up a straightforward way of thinking about things that can make future designs easier to reason about.  Following through a very basic example, we end with additional discussion of design considerations for other things like time-varying filters and extended filters for nonlinear systems. 

# Estimating Centripetal Acceleration

Using what we learned from figuring out how to use and design Kalman filters, we derive a straightforward way to build a Kalman filter to estimate centripetal acceleration for us in auto insurance telematics applications.  We discuss how to handle measurement error when the phone is loose in the car, or suffering from other extreme defects such as using the phone while driving.  Implementation considerations are discussed given how difficult it can be to get precise control of sensor sample rates on a phone.