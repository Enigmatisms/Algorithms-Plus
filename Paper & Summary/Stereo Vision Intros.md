# ML Intros & SLAM Basics

### SLAM Basic
- Base line: the line which connects 2 optical center of the stereo cams
  - Bigger the base line length, further the maximum detection range.
  - Of course, disparity can't be too large (or the object moves out of the optical plane) 
- Epipolar plane: 3 dots contraints:
  - 2 optical centers, `O1`, `O2`
  - 2 2-D points, 1 per cam respectively
  - with one optical center and 1 2-D dot we derive a line.
  - Two lines converge at one dot `P` 
  - The plane defined by `O1`, `O2`, `P`
- **Epipolar line**:
  - The lines where epipolar plane and the optical plane intersects.
- **Epipolar Contraint**:
  - This is stated clearly in the ppt.
  - About where the projection of one dot should be on 2 different optical planes.
- Reference view or plane:
  - If mono-cam: The prior frame should be the ref and the post frame will be target.
  - First we found the feature on the ref plane(view), then we want to extract the same feature dot on the target plane.

### Stereo system
- Offline calibration
- Rectification
  - Epipolar line should be matched and lined up.
  - Distortion should be removed.
- Preprocessing: like edge extraction or feature point description.
  - Methods like: decentralization -> bilateral filtering -> census tranformation
  - Other descriptor methods
  - Since two projected points should be on the same vertical posittion, we can search for it along the epipolar line.
- **Global and Local Algorithms**:
  - Global
    - Assign disparity to sub regions of the disparity image.
    - The assignment strategy should minimize the cost function (**Energe function**)
    - The energe function should be modeled.
    - Or using belief propagation.
- Prefix sum (Integral image) and Box Filter implementation [TODO]
- About box filtering in Stereo optimization:
  - Data reuse is repeated.
  - Except from the four corners, all the summation is done last time.
  - Therefore, we only have to do a massive calculation for the initialization.
### Cost Aggregation and window method
- Fixed window (FW):
  - The window is symmetric, of which the `center` is the central pos.
  - This could cause a lot of problem:
    - Small depth discontinuity areas: if the window is too large, over-padding or under padding might occur.
    - Not good at handling the repeatative or uniform area.
    - To fix this, the shape and size of the centered window should be adaptive.
- Shiftable windows:
  - Windows will not be centrally centered, different ROI will be extracted and voted.
- Multiple windows:
  - The window doesn't have to be rectangular.
  - The window consists of 9 / 25 sub windows, and whether some of them are present or not is decided by the score calculated.

### General Procedure for stereo vision
1. Stereo cam optaining two frames
2. Rectification.
3. Feature **matching** using: feature descriptor matching or epipolar constraints.
4. Solving the **correspondence** problems with multiple methods.
   1. Cost computation
   2. Cost aggregation
   3. Disparity optimization
   4. Disparity refinement
5. The Features are corresponded, with two frames mathced.
6. **Disparity computation and disparity optimization.**
7. Outlier detection and edge detection? (optional)
8. Yield the disparity result.

### Multiple methods for Cost Aggregation
#### The purpose for aggregation is to unify the similar region, remove noise inflence.
- Fixed windows (The easiest of all), one symmetric ROI with fixed size and shape, do the searching along the epipolar line.
- Other window methods:
  - shiftable (non-centrally-centered)
  - variable (size and center changable)
  - multiple (small sub windows, making the shape asymmetric)
- Segmentation (asymmetric and irregular with smoothness assumption)
- Bilateral filtering (BF): edge-preserving image filtering method
- Adaptive weights: slow
- Segment support: symmetric support, with weights only related to segmentation, etc.
- **Locally Consistent(LC)**
  - Modeling contiuit contraints (nearby-pixel relationship is considered)

### Multiple methods for Disparity Optimization
#### The purpose is finding the best disparity batches assignment
- Energy function minimization
  - Data term: how well it fits the truth
  - Smooth term: Noise robustness. Disparity within the same region should change continuously. Discontiuity will be panelized.
- Graph Cuts
- BP (**Will be closed examined in that Paper by Jian Sun et al.**)
- Cooperative optimization
- DP
- **SO(Scanline Optimization)**
  - The definition of cost is interesting and easy to understand.
- SO/DP/LC combined, etc.

### Multiple methods for Disparity Refinement
#### The purpose is to do outlier detection and subpixel assignment refining
- **Easy: Subpixel interpolation**
- Image filtering
- **Segmentation based outlier identification and replacement**
  - This is interesting because segment is approximated by a plane (3D, not frontal parallel)
  - Using RANSAC / Histogram voting (**An interesting 3D Hough-tranformation like method**)

#### TODO
##### Do some implementation practice
- [ ] Dynamic programming inrodcution implementation in Python
- [ ] Bilateral filtering in Python
- [ ] Prefix summation in CUDA C++
- [ ] Box filtering in CUDA C++