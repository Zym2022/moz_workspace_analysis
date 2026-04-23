Manipulability Analysis
Nikolaus Vahrenkamp∗†, Tamim Asfour∗, Giorgio Metta†, Giulio Sandini† and Ru¨diger Dillmann∗
∗ Institute for Anthropomatics, Karlsruhe Institute of Technology (KIT), Karlsruhe, Germany
† Robotics, Brain and Cognitive Sciences Departement, Istituto Italiano di Tecnologia (IIT), Genova, Italy
Abstract—The ability of computing a quality index for
given configurations can be useful for several applications in
the context of robotic manipulation. E.g. it can be used for
monitoring the current state of the system or it can support
decisionprocesses,suchasgraspselectioninhumanoidrobotics.
Here, a large set of precomputed grasps for a given object
have to be quickly filtered in order to select the reachable sub
set, for which the inverse kinematics (IK) problem has to be
solved. In this work, we present an approach for analyzing
the workspace capabilities of a manipulator in order to store
a representation for efficient online processing. Compared to Fig.1. ThehumanoidrobotsiCubandArmar-IIIperforminggraspingtasks
existing work, where usually reachability information is used inclutteredenvironments.
to represent the robot’s capabilities, we show how an extended
manipulabilitymeasurecanbeusedtobuildaqualitydistribu- support grasp selection in manipulation tasks (see Fig. 1) in
tion in workspace. The proposed approach for manipulability ordertofavorgraspsthatresultinahighmanipulability.High
measurement is suitable for redundant manipulators while
manipulability allows to adapt movements during execution,
considering joint limits and self-distance. Further, task specific
e.g. when visual servoing approaches are used to improve
distributions are introduced and several applications for the
humanoid robots ARMAR-III [1] and iCub [2] are presented. the accuracy [7]. The proposed algorithms are published
within the open-source robotics toolbox Simox, which can
I. INTRODUCTION be accessed online1.
The ability of supervising own movements is essential for
A. Manipulability Measure
robots operating in such different domains as industrial ap-
plications, service or humanoid robotics. Hence, the systems With Yoshikawa’s manipulability index [3] a quality mea-
must be equipped with methods to measure their perfor- sure for redundant manipulators was introduced, which de-
mance, accuracy or success rate. In this context, measuring scribes the distance to singular configurations. The approach
themanipulabilityisawell-knowntechniquefordetermining is based on analyzing the manipulability ellipsoid that is
the ability to maneuver in workspace. spanned by the singular vectors of the Jacobian. In [3] the
manipulability measure is proposed as
The manipulability index of given robot poses, introduced
by Yoshikawa in 1985 [3], can be calculated as a quality (cid:113)
w = det(JJT)=s s ···s . (1)
value that gives information about how good an adjustment 1 2 n
inworkspaceispossible.Withsuchinformation,redundancy Since w can be rewritten by multiplying the singular
of the robot can be exploited to favor poses which result in values s , the measure is proportional to the volume of
i
highermanipulabilityandwhichhenceallowbetteradaption the manipulability ellipsoid. The inverse condition number
during execution. In case the manipulability is used to measurement was introduced in [8] as the relation of the
support a decision process, e.g. when a grasp should be smallest to the largest singular value
selected out of a set of potential grasps, a precomputed
1 s
representation can be useful in order to serve manipulability c= = n. (2)
cond(J) s
queries efficiently. 1
Several approaches of determining the reachability of In [9], the following penalization was introduced to con-
manipulators are known in literature, which are based on sider the distance to the lower (l−) and the upper (l+) joint
j j
the idea that a voxelized 3D or 6D grid is filled with reach- limits:
a re b p il r i e t s y en in ta fo ti r o m n at t i h o e n r ( e s a e c e h [ a 4 b ] i , li [ ty 5], en [6 tr ] i ) e . s D c u a e n to jus th t e g d iv i e scr a et h iz in e t d , P(θ )=1−exp(−k (cid:89) n (θ j −l j −)(l j +−θ j ) ). (3)
j (l+−l−)2
whether a pose in workspace will lead to a valid IK solution j=1 j j
ornot.Inthisworkwewillshow,howasimilardatastructure
Inthispenaltyfunctionk isascalingfactorthatcanbeused
can be used to capture the manipulability distribution of
to adjust the behavior near joint limits. By multiplying the
a manipulator in workspace. The resulting data therefore
manipulability with P, configurations which are near joint
extends the approximated reachability information with a
quality index. This additional information can be used to 1http://simox.sourceforge.net

limitsarepenalized,butredundancyisnotconsidered,which control strategy for a requested end effector movement, but
canleadtoanerroneousqualityinformation(e.g.onejointis we want to investigate the limitations that are given for any
close to a boundary while the end effector’s maneuverability possible movements in workspace.
| is not affected |          | due to  | redundancy). |     |             |           |     |          |       |              |     |          |     |     |     |
| --------------- | -------- | ------- | ------------ | --- | ----------- | --------- | --- | -------- | ----- | ------------ | --- | -------- | --- | --- | --- |
|                 |          |         |              |     |             |           |     | A. Joint | Limit | Penalization |     | Function |     |     |     |
| A task          | specific | quality | function     | for | a redundant | manipula- |     |          |       |              |     |          |     |     |     |
tor is considered in [10] in order to compute optimal poses. Theinfluenceofjointlimitsonthemanipulabilitymeasure
Adexterousperformancemeasureispresentedin[11].By can be considered by analyzing a modified Jacobian matrix
analyzing the so-called augmented Jacobian matrix, which J. The entries of J are penalized according to the quality
| combines | information |     | about | the position, |     | orientation, | and |         |      |             |     |         |          |          |        |
| -------- | ----------- | --- | ----- | ------------- | --- | ------------ | --- | ------- | ---- | ----------- | --- | ------- | -------- | -------- | ------ |
|          |             |     |       |               |     |              |     | measure | that | arises from | the | current | distance | to joint | limits |
joint limits of the end-effector, the manipulability of joint (l− and l+). Since the distance to lower or upper joint
|     |     |     |     |     |     |     |     | j   | j   |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
configurations can be measured. It is shown that this ap- limit usually differs, the corresponding penalization terms
proach is more accurate than Yoshikawa’s manipulability also differ and hence, the quality measure depends on the
measure and exemplary applications for optimizing base investigated workspace movement.
| placements | are | given. | However, | the | approach | relies | on the |         |         |       |           |          |      |     |          |
| ---------- | --- | ------ | -------- | --- | -------- | ------ | ------ | ------- | ------- | ----- | --------- | -------- | ---- | --- | -------- |
|            |     |        |          |     |          |        |        | In [14] | a joint | limit | potential | function | h(θ) | is  | proposed |
generation of surface patches to represent the reachable and the joint limit gradient function (cid:53)h(θ) is derived. For
workspace, which can lead to challenging computational each joint θ the corresponding entry of (cid:53)h(θ) can be with
j
| problems.  | Joint       | velocity | limits      | can       | be considered | [12], | re- |              |     |       |     |           |     |        |       |
| ---------- | ----------- | -------- | ----------- | --------- | ------------- | ----- | --- | ------------ | --- | ----- | --- | --------- | --- | ------ | ----- |
|            |             |          |             |           |               |       |     |              |     |       | (θ  | −l −)2(2θ | −l  | +−l −) |       |
| sulting in | an analysis |          | of a scaled | Jacobian. |               |       |     |              |     | ∂h(θ) |     | j j       | j   | j j    |       |
|            |             |          |             |           |               |       |     | (cid:53)h(θ) | =   |       | =   |           |     |        | . (4) |
As shown in [12] or [13], the use of manipulability j ∂θ 4(l+−θ )2(θ −l−)2
|               |     |            |        |            |            |                |        |               |     | j        |          | j      | j j   | j         |        |
| ------------- | --- | ---------- | ------ | ---------- | ---------- | -------------- | ------ | ------------- | --- | -------- | -------- | ------ | ----- | --------- | ------ |
| ellipsoids    | can | lead to    | issues | that       | are caused | by             | mixing |               |     |          |          |        |       |           |        |
|               |     |            |        |            |            |                |        | This gradient |     | is equal | to zero, | if the | joint | is at the | middle |
| translational | and | rotational |        | sub spaces | for        | manipulability |        |               |     |          |          |        |       |           |        |
measurement. By using manipulability polytopes the de- of its range and goes to infinity at either limit. It can be
|         |          |     |             |     |            |               |     | used to | avoid | joint limits | during | online | control | as  | shown in |
| ------- | -------- | --- | ----------- | --- | ---------- | ------------- | --- | ------- | ----- | ------------ | ------ | ------ | ------- | --- | -------- |
| scribed | problems | can | be avoided, |     | but higher | computational |     |         |       |              |        |        |         |     |          |
costs have to be taken into account. In this work, we use [14] or [15]. In order to construct an according penalization
termwehavetodistinguishthedifferentpotentialmovement
| a weighting | matrix | W            | x in | order  | to set      | the translational |     |            |     |           |      |             |           |     |        |
| ----------- | ------ | ------------ | ---- | ------ | ----------- | ----------------- | --- | ---------- | --- | --------- | ---- | ----------- | --------- | --- | ------ |
|             |        |              |      |        |             |                   |     | directions | in  | workspace | (see | Sec. II-C). | Depending |     | of the |
| components  | of     | the Jacobian |      | matrix | in relation | with              | the |            |     |           |      |             |           |     |        |
rotational components (see [13]). actualpositionofthejoint(eitherlocatedinthelowerorthe
upperhalfofitsrange),thefollowingtwopenalizationterms
| B. Precomputed |     | Reachability |     | Information |     |     |     | are built: |     |     |     |     |     |     |     |
| -------------- | --- | ------------ | --- | ----------- | --- | --- | --- | ---------- | --- | --- | --- | --- | --- | --- | --- |
In literature the reachability of a manipulator is loosely (cid:40) −l−|>|l+−θ
|     |     |     |     |     |     |     |     |     |     |     | 1,  | |θ  |     | |   |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|     |     |     |     |     |     |     |     | p−  |     |     |     | j   | j   | j j |     |
defined as the volume of the workspace that can be reached = √ 1 , ot herw ise
j
by the end effector. Usually the three- or six-dimensional 1+|(cid:53) h(θ)j|
|     |     |     |     |     |     |     |     |     | (cid:40) |     |     | −l−|>|l+−θ |     |     | (5) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | -------- | --- | --- | ---------- | --- | --- | --- |
|     |     |     |     |     |     |     |     |     |          | √ 1 | ,   | |θ         |     | |   |     |
Cartesian space around the robot is investigated, whereas p+ 1+|(cid:53)h(θ)j| j j j j
=
only the position (3D) or position and orientation (6D) is j 1, otherwise
| considered. | The | reachability |     | data | is created | by  | filling | a      |        |              |     |         |        |     |         |
| ----------- | --- | ------------ | --- | ---- | ---------- | --- | ------- | ------ | ------ | ------------ | --- | ------- | ------ | --- | ------- |
|             |     |              |     |      |            |     |         | In Eq. | 5, the | penalization |     | term p− | stands | for | the pe- |
voxelized data structure in an offline step. The grid entries j
representeitheraprobabilitythatagivenposethatlieswithin nalization that has to be applied when investigating the
|                   |     |           |     |       |              |     |        | joint’s | movement | in  | negative | direction. | If  | the current | joint |
| ----------------- | --- | --------- | --- | ----- | ------------ | --- | ------ | ------- | -------- | --- | -------- | ---------- | --- | ----------- | ----- |
| the corresponding |     | workspace |     | voxel | is reachable | or  | binary |         |          |     |          |            |     |             |       |
informationisusedtoindicatethatavoxellies(partly)within value is located in the upper half of its range, the value
the reachable workspace (see [4], [5], [6]). is 1, which means that the movement does not underlie
|         |      |     |         |     |         |        |        | any penalizations. |     | Otherwise, |     | the | movement | is  | penalized |
| ------- | ---- | --- | ------- | --- | ------- | ------ | ------ | ------------------ | --- | ---------- | --- | --- | -------- | --- | --------- |
| In this | work | we  | pick up | the | idea of | having | a pre- |                    |     |            |     |     |          |     |           |
computedworkspacerepresentationofamanipulator.Instead according to Eq. 4. The second part of Eq. 5, describes the
|            |             |     |       |     |               |     |       | constructionofp+ |     | ,thepenalizationtermthatisappliedwhen |     |     |     |     |     |
| ---------- | ----------- | --- | ----- | --- | ------------- | --- | ----- | ---------------- | --- | ------------------------------------- | --- | --- | --- | --- | --- |
| of holding | information |     | about | the | reachability, | we  | store | a                |     | j                                     |     |     |     |     |     |
qualityindexthatservesinformationaboutthemanipulability amovementinpositivejointdirectionisinvestigated.Inthis
in workspace (see Sec. III). case, the penalization is only considered when the current
|         |                               |        |             |     |                |          |     | joint position |              | is located | in the   | upper | half of | its range. |     |
| ------- | ----------------------------- | ------ | ----------- | --- | -------------- | -------- | --- | -------------- | ------------ | ---------- | -------- | ----- | ------- | ---------- | --- |
| II.     | EXTENDEDMANIPULABILITYMEASURE |        |             |     |                |          |     |                |              |            |          |       |         |            |     |
|         |                               |        |             |     |                |          |     | B. Obstacle    | Penalization |            | Function |       |         |            |     |
| In this | work                          | we use | an extended |     | manipulability | measure- |     |                |              |            |          |       |         |            |     |
mentinordertoconsiderconstraintsthatlimitthemaneuver- Obstacles, such as environmental objects or parts of the
abilityinworkspace.Suchconstraintsareintroducedbyjoint robot, limit the possibility of a manipulator to maneuver
boundaries and workspace (self-)collisions, but any other in workspace. Hence, the manipulability measure should
constraintscanbeincorporatedaslongasthederivationwith consider a penalization that is derived from the distance and
respect to joint movements can be built. the position of any limiting obstacles. By determining the
In the context of optimal control, several approaches are nearest points p and p on the surface of the obstacle
|     |     |     |     |     |     |     |     |     |     | o   | m   |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
known to avoid joint limits and obstacles [14], [15]. Com- and the manipulator, one can determine the corresponding
v(cid:48)
paredtotheseworks,wearenotinterestedtofindanoptimal obstacle vector = p o − p m and the obstacle distance

d=|v(cid:48)|. The vector v ∈R3 is extended to R6 by setting the
rotational components to zero: v =[v ,v ,v ,0,0,0]T.
0 1 2
A collision function P(θ,d) should go to infinity for
d → 0 and decays exponentially to zero as d increases. As
proposed in [15], we choose
P(θ,d)=e−αdd−β. (6)
The parameters α and β can be used to adjust the obstacle
influence (see [15] for details). The gradient of P is the
Fig.2. (a)Theclassicalmanipulabilityellipsoidwithoutjointlimitpenal-
collision gradient function, which gives information about ization(c=0.170).(b)Foreachquadrantdifferentpenalizationtermsare
how each joint influences the obstacle distance: appliedaccordingtothecorrespondingjointlimitinfluence(c˜ext=0.047).
(cid:20) (cid:21)
(c)Theobstacleinfluencesthepenalizationterms(c˜ext=0.067).
∂P ∂P ∂P ∂P ∂d
(cid:53)P(θ,d)= = ,..., = . (7)
∂θ ∂θ ∂θ ∂d ∂θ joint limit) can be applied in order to penalize the computed
1 n
maneuverability.Therefore,thefollowingpenalizationmatrix
As shown in [15] (cid:53)P can be computed with
is computed:
∂P =−e−αdd−β(βd−1+α),
∂d (8) (cid:26) p−, sgn(J (θ))sgn(Γ )<0
∂d = 1(cid:2) JTv (cid:3)T . L (Γ,θ)= j i,j i (10)
∂θ d i,j p+, otherwise
j
Finally, two penalization terms can be constructed similar to
The selection which penalization has to be applied depends
Eq.5.Again,weareinterestedinallpotentialmovementsof
on two facts. Firstly, the sign of the corresponding Jacobian
the end effector and therefore a distinction is made, whether
entry J indicates the direction a positive Cartesian move-
a workspace movement in negative (o− ) or positive (o+ ) i,j
i,j i,j ment causes in joint space. Further, the sign of i-th entry of
direction is investigated:
Γ indicates which hyperoctant is considered. E.g. a positive
(cid:40)
1, v >0,i>3 sign of J and a hyperoctant in a negative coordinate axis
o− i,j = √ 1 , o i therwise results in i a ,j joint space movement towards the negative joint
(cid:40) √ 1+ 1 |(cid:53)Pj| , v >0,i≤3 (9) limit and hence, p− j has to be considered for penalization.
o+ = 1+|(cid:53)Pj| i 2. Collision Penalization Matrix
i,j
1, otherwise
Inaddition,collisionpenalizationscanbecomputed.Here,
In Eq. 9 the terms o− i,j and o+ i,j represent the penalizations either o− i (movement in negative coordinate axis) or o+ i
thathavetobeappliedwheninvestigatingmovementsinneg- (movement in positive coordinate axis) are considered:
ativerespectivelypositiveCartesiandirections.Penalizations (cid:26) o− , sgn(Γ )<0
are only applied for translational dimensions (i ≤ 3) and if O i,j (Γ,θ)= o i + ,j , otherw i ise (11)
i,j
the obstacle is located in the corresponding direction (v ).
i
3. The Augmented Jacobian
C. Computing the Augmented Jacobians Finally, the augmented Jacobian J˜ is constructed as fol-
Compared to [14] or [15], we cannot use the gradients lows:
directly since an explicit movement x ∈ R6 is needed in J˜ (Γ,θ)=L (Γ,θ)O (Γ,θ)J (θ). (12)
i,j i,j i,j i,j
order to optimize the control law. In the context of ma-
nipulability, we are interested in a measurement that serves By applying Eq. 12 to all possible permutations of Γ,
information about the possibility to maneuver in workspace 64 Jacobians are generated, each of which describes the
and hence all potential movements must be covered. Since maneuverability in the corresponding hyperoctant of the
the manipulability is affected by joint limit and obstacle workspace. These matrices can be analyzed for manipu-
constraints and, as shown above, these constraints directly lability in order to retrieve a quality index for a given
depend on the investigated movement, we have to build a configuration. Therefore all Jacobians are decomposed via
representation that covers all potential directions. Therefore, Singular Value Decomposition (SVD) in order to retrieve
the space of workspace movements is partitioned by 26 the corresponding sets of singular values s˜ Γ . To compute
hyperoctants, which are identified by Γ ∈ {−1,+1}6. the Extended Inverted Condition Number the minimum and
For each Γ an augmented Jabobian matrix J˜ is built by maximum singular value is determined:
considering the corresponding penalization terms. Finally,
themanipulabilityofJ˜iscomputed,whichgivesinformation c˜
ext
=
m
m
a
in
x
{
{
s
s
˜
˜
Γ
Γ
}
}
. (13)
aboutthemanipulabilityofallpotentialmovementdirections
The quality measure c˜ implicitly considers redundancy,
ext
that are represented by the corresponding hyperoctant Γ.
since a penalization due to joint limits can be compensated
1. Joint Limit Penalization Matrix by redundant joints in the kinematic structure. The results
Depending on the investigated movement of θ j , either p− j of this measure applied to a 4 degrees of freedom (DoF)
(movetowardslowerjointlimit)orp+ (movetowardsupper planar manipulator can be seen in Fig. 2. The figure on the
j

Fig. 4. (a) The task specific manipulability distribution for an upright
Fig.3. Toprow:(a)Thedistributionoftheclassicalmanipulabilitymeasure
prototype direction without considering joint limits (top). The distribution
isanalyzedforaplanar4DoFrobot.(b)Byapplyingaglobalpenalization
term,jointlimitscanbeconsidered,butredundancyisnotreflectedbythe
of the task specific manipulability c˜p (bottom). (b) The task-specific
manipulability distribution for the left arm of ARMAR-III considering an
measure. Bottom row:(c) The distribution of the extended manipulability
uprightliftingmovement.
measurec˜ext inworkspace.(d)Theobstacleisconsideredadditionally.
left shows a 2D visualization of the manipulability ellipsoid by computing the forward kinematics of the manipulator in
without considering joint limits, whereas the second visual- order to determine the location of the end effector. If the
ization shows the scaled manipulability ellipsoids in each of actual entry of v is lower than c˜ ext , the value is updated.
thefourquadrants.Ontheright,anobstacleisconsideredand Thisleadstoanupper-boundrepresentationoftheachievable
theminimumdistanceisvisualized.Theaccordingjointlimit manipulability which can be seen as an optimistic view on
and obstacle penalization terms are applied as described in the robot’s possibility to maneuver in workspace.
Eq. 12. Note, that only parts of the manipulability ellipsoids Fig. 3 shows a visualization of the manipulability distri-
are visualized, so that one can see the penalization effects in bution of a 2D planar robot with 4 DoF. For each position
the different quadrants. Due to this visualization technique, that is indicated, the maximum manipulability is shown,
discontinuous transitions arise between the quadrants. Such whereas the color indicates the magnitude of the achiev-
artifacts only affect the visualization and have no influence able manipulability (blue:low, red:high). On the top left
on the computation, since for each quadrant (respectively the classical manipulability index c is used, whereas the
hyperoctant in the general case) the whole manipulability top right figure shows the distribution when a penalization
ellipsoid is considered. term according to Eq. 3 is applied when measuring the
manipulability. Since redundancy is not considered in Eq. 3,
III. MANIPULABILITYANALYSIS
theresultingdistributiongivesonlylimitedinformationabout
In order to build up a representation of the manipulability the achievable manipulability of a redundant manipulator.
distribution in workspace, we propose a 6D voxelized data When applying the proposed penalizations, the corre-
structure holding manipulability reference values in each sponding distribution can be seen in Fig. 3. The bottom
voxel. Since the mapping between C-space and workspace left figure shows the result when joint limits are penalized
is not unique for redundant manipulators, either the average according to Eq. 10. On thebottom right figure, the obstacle
manipulability or the lower respectively the upper bound of penalization (Eq. 11) is additionally applied and the com-
the achievable manipulability within a voxel can be stored. bined Jacobians are analyzed as described in Section II-C.
Hence, the manipulability data cannot be seen as an exact
representation of the manipulators capabilities, but it can be
A. Task-Specific Manipulability Distributions
used to efficiently serve an approximation of the expected
manipulability of a given pose in workspace. This can be In case the robot is supposed to frequently execute a spe-
usefulinseveralsituations,e.g.whentheselectionofagrasp cific task, as it is the case e.g. for pick-and place operations,
should be made and a large number of potential grasping task-specificmanipulabilitydistributionscanbeprecomputed
poses are available. Further, the expected manipulability can in order to support the execution. Such distributions give
help in choosing a suitable location for a hand-over process. information about the possibility to move in a specific
A workspace representation of the robot’s reachability or direction v p ∈R6.
manipulability capabilities can be built by two ways. Either The penalization terms are computed according to Eq. 12,
work or joint space is discretized and all configurations are and since a hyperoctant (cid:15) p is defined by the direction v p ,
processed in order to update the corresponding 6D voxel, or only one augmented Jacobian J˜((cid:15) p ,θ) has to be assembled.
randomized approaches are used. In the following, we build Further,themagnitudeoftheJacobian’sgradientwithrespect
the manipulability representation by sampling a large set of tothegivenprototypedirectionv p isusedasaqualityindex:
configurationsrandomly.Foreachsample,thecorresponding
c˜ ext value is computed and the 6D voxel v is determined q p (θ)=|J˜((cid:15) p ,θ)Tv p |. (14)

|         |          |               |     |                       |     |              |     | Fig.6. (a)Themanipulabilityofasetofreachablegraspsisencodedby |     |     |     |     |     |     |
| ------- | -------- | ------------- | --- | --------------------- | --- | ------------ | --- | ------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| Fig. 5. | (a) A 3D | visualization | of  | the 6D manipulability |     | distribution |     |                                                               |     |     |     |     |     |     |
of iCub’s 10 DoF kinematic chain covering torso and right arm. (b) color. (b) A grasp with c˜ext = 0.02. (c) The optimal grasp in terms of
A cut through the manipulability distribution of the ARMAR-III’s right manipulability(c˜ext=0.12).
| manipulator | (torso and | arm). | Joint limits | and self-distance |     | are considered |     |     |     |     |     |     |     |     |
| ----------- | ---------- | ----- | ------------ | ----------------- | --- | -------------- | --- | --- | --- | --- | --- | --- | --- | --- |
bytheunderlyingmanipulabilitymeasure. for the corresponding joint configuration in order to deter-
|     |     |     |     |     |     |     |     | mine the | grasp | with maximum | c˜  | . In the | second | approach |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | ----- | ------------ | --- | -------- | ------ | -------- |
ext
Together with the inverted condition number c˜ for the all grasps are filtered according to their reachability. Here,
(cid:15)p
corresponding hyperoctant, the resulting manipulability can reachability is given when the corresponding entry of the
be computed with: manipulability distribution is greater zero. This leads to a
|     |     |     |          |     |     |     |      | binary representation |     | of the | 6D reachability |     | similar | to the |
| --- | --- | --- | -------- | --- | --- | --- | ---- | --------------------- | --- | ------ | --------------- | --- | ------- | ------ |
|     |     | c˜  | =q (θ)c˜ | .   |     |     | (15) |                       |     |        |                 |     |         |        |
p p (cid:15)p approaches of [4], [5], [6]. For the remaining sub-set of
The distribution of the corresponding manipulability val- reachable grasps, the IK-solutions are computed in order
ues when considering an upright prototype direction v can to determine the corresponding manipulability. The third
p
be seen in Fig. 4(a). The distribution of the manipulability approach takes full advantage of the precomputed manip-
|           |                  |     |     |          |        |           |     | ulability | data. Due | to the efficient |     | data structures |     | an ordered |
| --------- | ---------------- | --- | --- | -------- | ------ | --------- | --- | --------- | --------- | ---------------- | --- | --------------- | --- | ---------- |
| w.r.t. an | upright movement |     | for | the left | arm of | ARMAR-III |     |           |           |                  |     |                 |     |            |
is shown in Fig. 4(b). set of reachable grasps with corresponding manipulability
|                   |        |               |              |          |                |     |      | values can | be retrieved | in         | less than | one millisecond |       | and the       |
| ----------------- | ------ | ------------- | ------------ | -------- | -------------- | --- | ---- | ---------- | ------------ | ---------- | --------- | --------------- | ----- | ------------- |
|                   |        | IV.           | APPLICATIONS |          |                |     |      |            |              |            |           |                 |       |               |
|                   |        |               |              |          |                |     |      | IK-problem | has          | only to be | solved    | for the         | first | entry of this |
| A. Manipulability |        | Distribution  | of           | Humanoid | Robots         |     |      | list (see  | Table I).    |            |           |                 |       |               |
| In Fig.           | 5 a 3D | visualization |              | of the   | manipulability |     | dis- |            |              |            |           |                 |       |               |
TABLEI
| tributions | of iCub | and | ARMAR-III | can | be  | seen. In | both |     |     |     |     |     |     |     |
| ---------- | ------- | --- | --------- | --- | --- | -------- | ---- | --- | --- | --- | --- | --- | --- | --- |
COMPARISONOFDIFFERENTAPPROACHESFORGRASPSELECTION
casesthekinematicchaincoveringthreetorsoandsevenarm
(AVERAGEOF100TESTS).
| joints has | been considered. |     | The | underlying | manipulability |     |     |     |     |     |     |     |     |     |
| ---------- | ---------------- | --- | --- | ---------- | -------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
measure incorporates penalizations due to joint limits and Overall Reach./ IK Manip.
|                                                      |                 |     |         |         |                |       |     |               |     |       | Manip.access |     | Solving | Comp. |
| ---------------------------------------------------- | --------------- | --- | ------- | ------- | -------------- | ----- | --- | ------------- | --- | ----- | ------------ | --- | ------- | ----- |
| self distance                                        | as described    |     | in Sec. | II. The | left figure    | shows | a   | Noprecomputed |     |       |              |     |         |       |
|                                                      |                 |     |         |         |                |       |     |               |     | 9.97s | -            |     | 9.07s   | 0.90s |
| 3Dvisualizationofthe6Ddistribution,whichwasgenerated |                 |     |         |         |                |       |     | data          |     |       |              |     |         |       |
| by showing                                           | the orientation |     | with    | maximum | manipulability |       |     | Reachability  |     |       |              |     |         |       |
|                                                      |                 |     |         |         |                |       |     |               |     | 2.24s | <0.001s      |     | 1.75s   | 0.49s |
data
| at each        | 3D position. | On  | the right, | a cut | through | the | 6D  |                |           |       |                   |     |       |     |
| -------------- | ------------ | --- | ---------- | ----- | ------- | --- | --- | -------------- | --------- | ----- | ----------------- | --- | ----- | --- |
|                |              |     |            |       |         |     |     | Manipulability |           | 0.01s | <0.001s           |     | 0.01s | -   |
| distribution   | is shown.    |     |            |       |         |     |     | data           |           |       |                   |     |       |     |
| B. Performance | Evaluation   |     |            |       |         |     |     |                |           |       |                   |     |       |     |
|                |              |     |            |       |         |     |     | C. Grasp       | Selection | based | on Manipulability |     |       |     |
Themainadvantageofanofflinegeneratedmanipulability
distribution is given by the possibility to structure the data In this setup, manipulability distributions are used to
in order to speed up information retrieval during online support the grasp selection process, which is part of a
processing. In the following experiment we evaluate the grasping pipeline implemented on the humanoid robot iCub.
effect of using precomputed manipulability representations The grasping pipeline takes advantage of offline generated
in the context of grasp selection. data,suchasobjectspecificsetsofgraspsandmanipulability
In Fig. 6 the model of a water bottle for which 1000 data as proposed in this work. During online processing
precomputedgraspsareavailablecanbeseenontheleft.An a subset of reachable grasps is determined according to
exemplary subset of reachable grasps is depicted, whereas the object’s pose. Therefore, all grasps are transformed to
the manipulability is encoded by color (blue:low, red:high). Cartesian grasping poses by combining the object’s pose in
To measure the performance, the bottle is placed randomly workspace with the object-grasp relation. These poses can
in front of ARMAR-III and different strategies for grasp then be quickly filtered according to their manipulability
selection are evaluated. Firstly, no precomputed data is used and the resulting ordered subset gives information about
and the IK-problem is solved for all grasps for the 10 DoF reachability and manipulability of the remaining grasps. In
kinematic chain covering torso and arm. In case an IK- addition, the inverse kinematics (IK) problem is solved and
solution could be retrieved, the manipulability is computed a collision-free motion is generated with algorithms related

Fig.7. iCub’sgraspingpipelineissupportedbyprecomputedmanipulabilitydistributions.(a)Thevirtualrepresentationofthescene.(b)Allreachable
graspsarecolorizedaccordingtotheirmanipulability.(c)TheIK-solutionoftheselectedgrasp.(d)Acollision-freegraspingmotion.
Fig.8. ThegraspselectionofiCub’sgraspingpipelineissupportedbyprecomputedmanipulabilitydistributions.
to Rapidly-exploring Random Trees (RRT) [16]. Fig. 7 [4] F.Zacharias,C.Borst,andG.Hirzinger,“Capturingrobotworkspace
|           |             |          |                      |     |                | structure: | representing |            | robot capabilities,” | in            | Intelligent | Robots and |
| --------- | ----------- | -------- | -------------------- | --- | -------------- | ---------- | ------------ | ---------- | -------------------- | ------------- | ----------- | ---------- |
| depicts   | the results | of the   | different processing |     | stages and the |            |              |            |                      |               |             |            |
|           |             |          |                      |     |                | Systems,   | 2007.        | IROS 2007. | IEEE/RSJ             | International | Conference  | on,        |
| execution | on iCub     | is shown | in Fig.              | 8.  |                |            |              |            |                      |               |             |            |
292007-Nov.22007,pp.3229–3236.
|     |     |     |     |     |     | [5] R. Diankov, | “Automated |     | construction | of robotic | manipulation | pro- |
| --- | --- | --- | --- | --- | --- | --------------- | ---------- | --- | ------------ | ---------- | ------------ | ---- |
V. CONCLUSION grams,” Ph.D. dissertation, Robotics Institute, Carnegie Mellon Uni-
versity,Pittsburgh,PA,September2010.
In this work, we presented an integrated approach for [6] N. Vahrenkamp, D. Berenson, T. Asfour, J. Kuffner, and R. Dill-
determining the manipulability for redundant manipulators. mann, “Humanoid motion planning for dual-arm manipulation and
re-graspingtasks,”inIntelligentRobotsandSystems,IROS,2009.
We showed how classical measurements, which are mostly [7] N. Vahrenkamp, S. Wieland, P. Azad, D. Gonzalez, T. Asfour, and
based on analyzing the manipulability ellipsoid, can be R.Dillmann,“Visualservoingforhumanoidgraspingandmanipula-
tiontasks,”inHumanoidRobots,2008.Humanoids2008.8thIEEE-
| extended | in order | to consider | constraints | coming | from joint |     |     |     |     |     |     |     |
| -------- | -------- | ----------- | ----------- | ------ | ---------- | --- | --- | --- | --- | --- | --- | --- |
RASInternationalConferenceon,Dec.2008,pp.406–412.
| limits, workspace |     | obstacles | or self-distance |     | while explicitly |               |     |             |        |          |                     |     |
| ----------------- | --- | --------- | ---------------- | --- | ---------------- | ------------- | --- | ----------- | ------ | -------- | ------------------- | --- |
|                   |     |           |                  |     |                  | [8] M. Togai, | “An | application | of the | singular | value decomposition | to  |
considering redundancy. This extended manipulability mea- manipulabilityandsensitivityofindustrialrobots,”SIAMJournalon
AlgebraicandDiscreteMethods,vol.7,no.2,pp.315–320,1986.
| surement | is used | to build | the distribution | of  | a manipulator’s |     |     |     |     |     |     |     |
| -------- | ------- | -------- | ---------------- | --- | --------------- | --- | --- | --- | --- | --- | --- | --- |
[9] M.-J.Tsai,“Workspacegeometriccharacterizationandmanipulability
manipulabilityinworkspace,allowingtocapturethecapabil-
ofindustrialrobots,”Ph.D.dissertation,OhioStateUniversity,1986.
| ities of           | a robot | in terms     | of operational | freedom.    | In addition  |                 |            |           |                |                 |                  |        |
| ------------------ | ------- | ------------ | -------------- | ----------- | ------------ | --------------- | ---------- | --------- | -------------- | --------------- | ---------------- | ------ |
|                    |         |              |                |             |              | [10] B.-H.      | Jun, P.-M. | Lee,      | and J. Lee,    | “Manipulability | analysis         | of un- |
|                    |         |              |                |             |              | derwater        | robotic    | arms      | on rov and     | application     | to task-oriented | joint  |
| weshowed           | how     | taskspecific | datacan        | begenerated | andhow       |                 |            |           |                |                 |                  |        |
|                    |         |              |                |             |              | configuration,” |            | in OCEANS | ’04. MTTS/IEEE |                 | TECHNO-OCEAN     | ’04,   |
| the manipulability |         | data         | can be used    | for grasp   | selection on |                 |            |           |                |                 |                  |        |
vol.3,nov.2004,pp.1548–1553Vol.3.
humanoid robots. [11] K. Abdel-Malek, W. Yu, and J. Yang, “Placement of robot manipu-
|     |     |     |     |     |     | lators | to maximize | dexterity,” | International |     | Journal of | Robotics and |
| --- | --- | --- | --- | --- | --- | ------ | ----------- | ----------- | ------------- | --- | ---------- | ------------ |
Automation,vol.19,no.1,pp.6–14,2004.
ACKNOWLEDGMENT
[12] J.Lee,“Astudyonthemanipulabilitymeasuresforrobotmanipula-
tors,”inIntelligentRobotsandSystems,1997.IROS’97.,Proceedings
| The research |     | leading to | these results | has | received funding |     |     |     |     |     |     |     |
| ------------ | --- | ---------- | ------------- | --- | ---------------- | --- | --- | --- | --- | --- | --- | --- |
ofthe1997IEEE/RSJInternationalConferenceon,vol.3,sep1997,
from the European Union Seventh Framework Programme pp.1458–1465vol.3.
under grant agreement 270273 (Xperience). [13] R. Finotello, T. Grasso, G. Rossi, and A. Terribile, “Computation of
|     |     |     |     |     |     | kinetostatic | performances |     | of robot | manipulators | with | polytopes,” in |
| --- | --- | --- | --- | --- | --- | ------------ | ------------ | --- | -------- | ------------ | ---- | -------------- |
RoboticsandAutomation,1998.Proceedings.1998IEEEInternational
REFERENCES
Conferenceon,vol.4,may1998,pp.3241–3246vol.4.
|     |     |     |     |     |     | [14] T. F. | Chan and | R. Dubey, | “A weighted | least-norm | solution | based |
| --- | --- | --- | --- | --- | --- | ---------- | -------- | --------- | ----------- | ---------- | -------- | ----- |
[1] T. Asfour, K. Regenstein, P. Azad, J. Schro¨der, A. Bierbaum, scheme for avoiding joint limits for redundant joint manipulators,”
N. Vahrenkamp, and R. Dillmann, “ARMAR-III: An integrated hu- Robotics and Automation, IEEE Transactions on, vol. 11, no. 2, pp.
| manoid | platform | for sensory-motor | control.” | in  | IEEE-RAS Interna- |     |     |     |     |     |     |     |
| ------ | -------- | ----------------- | --------- | --- | ----------------- | --- | --- | --- | --- | --- | --- | --- |
286–292,1995.
tionalConferenceonHumanoidRobots,2006,pp.169–175.
|     |     |     |     |     |     | [15] B. Dariush, | G.  | Bin Hammam, | and | D. Orin, | “Constrained | resolved |
| --- | --- | --- | --- | --- | --- | ---------------- | --- | ----------- | --- | -------- | ------------ | -------- |
[2] G. Metta, G. Sandini, D. Vernon, L. Natale, and F. Nori, “The icub accelerationcontrolforhumanoids,”inIntelligentRobotsandSystems,
humanoidrobot:anopenplatformforresearchinembodiedcognition,” 2010IEEE/RSJInternationalConferenceon,oct.2010,pp.710–717.
in Proceedings of the 8th Workshop on Performance Metrics for [16] J. Kuffner and S. LaValle, “RRT-connect: An efficient approach to
| IntelligentSystems. |     | NewYork,NY,USA:ACM,2008,pp.50–56. |            |              |              |              |      |            |         |       |          |              |
| ------------------- | --- | --------------------------------- | ---------- | ------------ | ------------ | ------------ | ---- | ---------- | ------- | ----- | -------- | ------------ |
|                     |     |                                   |            |              |              | single-query | path | planning,” | in IEEE | Int’l | Conf. on | Robotics and |
| [3] T. Yoshikawa,   |     | “Manipulability                   | of robotic | mechanisms,” | The Interna- |              |      |            |         |       |          |              |
Automation(ICRA’2000),SanFrancisco,CA,2000,pp.995–1001.
tionalJournalofRoboticsResearch,vol.4,no.2,pp.3–9,1985.
