# COMP4034-Group11
Group 11

We also need to think about how to do an extensive evaluation for the report.

From the assessment doc:

<b> Evaluation.
  
Here the working system should be critically appraised and your report should include  evidence  of  thorough  functionality  testing,  assessment  of  the  system  limitations, description of possible improvements and checking the influence of the system parameters on the overall performance (e.g. robot speed, processing speed, controller values, etc.). Does your   approach   meet   the   original   objectives?   Please   comment   also   on   the   overall performance of your controller
</b>


Current ideas:

Total distance travelled.
Total time taken.

Time taken to find all targets after locating the first target?
Total distance travelled after locating the first target?

Time taken/distance travelled per target?

Identify possible "hyperparameters" and do trial runs with different configurations and report on performance.

Given that there will be a test environment, we could also make our own by moving around the obstacles and targets and seeing how it performs compared to the original training scene, so we are able to comment on generality of our approach in our report (prior to the actual demo).

I expect we could save it as a separate world file, and have a separate launch file associated with it.
