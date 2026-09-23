import os, sys, csv, json
from pathlib import Path
ROOT=Path(r'\\wsl.localhost\Ubuntu\home\satyam\ROHMPC-State-Dependent-Uncertainty')
WORK=ROOT/'work/pdf-report'; WORK.mkdir(parents=True,exist_ok=True)
os.environ['MPLCONFIGDIR']=str(WORK/'mplconfig')
sys.path.insert(0,str(ROOT/'work/pdf-deps'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.mathtext import math_to_image
from matplotlib.font_manager import FontProperties
from PIL import Image as PILImage
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image, Flowable, KeepTogether
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.enums import TA_LEFT
from pypdf import PdfReader
OUT=ROOT/'output/pdf'; OUT.mkdir(parents=True,exist_ok=True)
PDF=OUT/'ROHMPC_Offline_Certificates_and_State_Dependent_Bounds.pdf'
fontdir=Path(matplotlib.get_data_path())/'fonts/ttf'
for name,f in [('Body','DejaVuSans.ttf'),('Bold','DejaVuSans-Bold.ttf'),('Italic','DejaVuSans-Oblique.ttf')]:
    pdfmetrics.registerFont(TTFont(name,str(fontdir/f)))
pdfmetrics.registerFontFamily('Body',normal='Body',bold='Bold',italic='Italic',boldItalic='Bold')
NAVY=colors.HexColor('#17324D'); TEAL=colors.HexColor('#087F8C'); INK=colors.HexColor('#253746'); GRAY=colors.HexColor('#5C6D79'); LIGHT=colors.HexColor('#EDF5F7')
WIDTH=507.28
styles={
 'body':ParagraphStyle('body',fontName='Body',fontSize=10,leading=14.7,textColor=INK,spaceAfter=8),
 'small':ParagraphStyle('small',fontName='Body',fontSize=8.4,leading=12,textColor=GRAY,spaceAfter=6),
 'h1':ParagraphStyle('h1',fontName='Bold',fontSize=23,leading=29,textColor=NAVY,spaceAfter=12),
 'h2':ParagraphStyle('h2',fontName='Bold',fontSize=12,leading=17,textColor=NAVY,spaceBefore=7,spaceAfter=7),
 'kicker':ParagraphStyle('kicker',fontName='Bold',fontSize=8.5,leading=12,textColor=TEAL,spaceAfter=9),
 'table':ParagraphStyle('table',fontName='Body',fontSize=8.7,leading=12,textColor=INK),
 'th':ParagraphStyle('th',fontName='Bold',fontSize=8.6,leading=12,textColor=colors.white),
}
story=[]
def p(t,style='body'): return Paragraph(t,styles[style])
def add(t,style='body'): story.append(p(t,style))
def h(t): add(t,'h2')
def start(num,title,lead):
    if story: story.append(PageBreak())
    add(f'RESEARCH WALKTHROUGH  /  {num:02d}','kicker'); add(title,'h1'); add(lead)
def table(headers,rows,widths=None):
    data=[[p(x,'th') for x in headers]]+[[p(str(x),'table') for x in row] for row in rows]
    tb=Table(data,colWidths=widths or [WIDTH/len(headers)]*len(headers),hAlign='LEFT')
    tb.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),NAVY),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),9),('RIGHTPADDING',(0,0),(-1,-1),9),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.HexColor('#F1F5F8'),colors.white]),('LINEBELOW',(0,-1),(-1,-1),0.5,colors.HexColor('#D8E2E8'))]))
    story.append(tb); story.append(Spacer(1,9))
def note(title,text):
    tb=Table([[p('<b>'+title+'</b><br/>'+text)]],colWidths=[WIDTH])
    tb.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),LIGHT),('BOX',(0,0),(-1,-1),0.5,colors.HexColor('#BFDCE2')),('LEFTPADDING',(0,0),(-1,-1),12),('RIGHTPADDING',(0,0),(-1,-1),12),('TOPPADDING',(0,0),(-1,-1),10),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
    story.append(tb); story.append(Spacer(1,9))
eqcounter=0
def mathimage(s,maxwidth=WIDTH-35,fontsize=12):
    global eqcounter
    eqcounter+=1; path=WORK/f'eq_{eqcounter:03d}.png'
    math_to_image('$'+s+'$',str(path),prop=FontProperties(size=fontsize),dpi=240,format='png',color='#17324D')
    with PILImage.open(path) as im: w,hg=im.size
    w=w*72/240; hg=hg*72/240; scale=min(1,maxwidth/w)
    return Image(str(path),width=w*scale,height=hg*scale)
def eq(s,num=None):
    img=mathimage(s)
    tb=Table([[img,p(f'({num})' if num else '', 'small')]],colWidths=[WIDTH-28,28])
    tb.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'MIDDLE'),('ALIGN',(0,0),(0,0),'CENTER'),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),8)]))
    story.append(tb)
class BlockMatrix(Flowable):
    def __init__(self,label,cells):
        Flowable.__init__(self)
        self.label=mathimage(label,fontsize=12); self.cells=[[mathimage(c,fontsize=11) for c in row] for row in cells]
        self.cw=[max(row[j].drawWidth for row in self.cells)+20 for j in range(2)]
        self.rh=[max(im.drawHeight for im in row)+15 for row in self.cells]
        self.width=self.label.drawWidth+20+sum(self.cw)+16; self.height=sum(self.rh)+10
    def draw(self):
        c=self.canv; x=(WIDTH-self.width)/2; self.label.drawOn(c,x,(self.height-self.label.drawHeight)/2)
        left=x+self.label.drawWidth+12; right=left+sum(self.cw)+12
        c.setStrokeColor(NAVY); c.setLineWidth(0.9)
        for xx,sgn in [(left,1),(right,-1)]:
            c.line(xx,5,xx,self.height-5); c.line(xx,5,xx+5*sgn,5); c.line(xx,self.height-5,xx+5*sgn,self.height-5)
        top=self.height-5
        for i,row in enumerate(self.cells):
            xx=left+6
            for j,im in enumerate(row): im.drawOn(c,xx+(self.cw[j]-im.drawWidth)/2,top-self.rh[i]+(self.rh[i]-im.drawHeight)/2); xx+=self.cw[j]
            top-=self.rh[i]

def footer(c,doc):
    c.saveState(); c.setStrokeColor(colors.HexColor('#D4E0E6')); c.line(44,39,551,39)
    c.setFont('Body',8); c.setFillColor(GRAY); c.drawString(44,25,'ROHMPC  |  Offline design and state-dependent error bounds')
    c.drawRightString(551,25,f'{doc.page} / 10'); c.restoreState()

start(1,'From the baseline to\nstate-dependent error bounds','A step-by-step record of the work completed so far, with the mathematical derivation and a guide to the code. Prepared for Satyam Awasthi. Updated 11 September 2026.')
note('Current result','We recovered the original uncalibrated observer radius from its LMI and constructed a smaller local process envelope using the same metric and gains. This is supported by numerical audits; continuous-domain and full MPC guarantees remain open.')
h('The work in five steps')
table(['Step','What was done','Where explained'],[
['1. Reproduce','Initialized public dependencies and ran the two offline SDPs.','Page 3'],
['2. Understand','Separated true-to-estimate and estimate-to-nominal errors.','Pages 2 and 4'],
['3. Prototype','Tried dynamic norm bounds; found extra conservatism.','Page 4'],
['4. Recover','Derived squared-radius dynamics directly from the original LMI.','Page 5'],
['5. Localize','Reduced the disturbance envelope and compared tube propagation.','Pages 6-8']], [83,337,87])
h('What this report is about')
add('The focus is the offline observer/controller certificates and error-tube propagation. We have not yet run a closed-loop quadrotor simulation with the new tubes, integrated them into online MPC, or recomputed UQ for a physical system.')
h('How to read it')
add('Read pages 2-3 for the model and baseline. Pages 4-7 derive the new bounds. Page 8 explains the results, page 9 separates evidence from guarantees, and page 10 maps the equations to scripts.')
add('Source basis: the ROHMPC paper (arXiv:2508.07045v1), the pinned public mpc-sdp implementation, and our saved MATLAB runs. Precise source revisions and file locations are on page 10.','small')

start(2,'The model and the two errors','The observer, nominal prediction and true plant are different trajectories. Keeping them separate explains the two tubes.')
eq(r'\dot{x}=f_0(x,u)+E_0D(a(v))w^0,\qquad y=Cx+F\eta','1')
eq(r'\dot{\hat{x}}=f_0(\hat{x},u)+L(y-C\hat{x}),\quad \dot{z}=f_0(z,u_{\rm nom})','2')
eq(r'u=u_{\rm nom}+K(\hat{x}-z)','3')
add('Here f<sub>0</sub> includes the fixed modeled disturbance bias. The residual disturbance w<sup>0</sup> lies in the original centered box. D scales only acceleration channels; all other process channels and measurement-noise bounds are unchanged.')
table(['Symbol','Meaning'],[
['x, x-hat, z','True state, observer estimate, nominal trajectory.'],
['e = x - x-hat','Estimation error: what the observer does not know.'],
['delta = x-hat - z','Tracking error of the estimate relative to the nominal trajectory.'],
['P-delta','Positive-definite metric used to measure both errors.'],
['r(t), s(t)','Upper bounds on estimation and tracking errors in that metric.'],
['R(t) = r(t)<super>2</super>','Squared estimation radius, used in the improved derivation.'],
['T','Cholesky factor: T<super>T</super>T = P-delta. Not the scalar R.']], [142,365])
eq(r'\|e\|_{P^\delta}\leq r,\quad \|\delta\|_{P^\delta}\leq s,\quad \|x-z\|_{P^\delta}\leq r+s','4')
note('A radius is a bound, not an observed error','These are weighted state-space radii, not distances in meters. A smaller computed radius does not by itself mean the actual estimation error is smaller.')

start(3,'What the baseline run computed','An LMI is a matrix inequality. An SDP is an optimization problem containing semidefinite constraints, typically many LMIs.')
eq(r'M(q)=M_0+\sum_i q_iM_i\preceq0','5')
add('The inequality means every eigenvalue is nonpositive, or equivalently a<super>T</super>M(q)a is nonpositive for every a. It is not an entrywise inequality. YALMIP constructs the optimization problem; MOSEK solves it.')
table(['Offline problem','Computed ingredients'],[
['Robust-design SDP','X, Y, metric P-delta = X<super>-1</super>, gain K = YX<super>-1</super>, tightening coefficients, observer radius epsilon and auxiliary envelopes. The observer gain L is prescribed in this run.'],
['Terminal-cost SDP','A separate terminal-cost matrix P. This is different from the error metric P-delta.']], [145,362])
h('The actual 1,024-point operating grid')
table(['Quantity','Values used'],[
['Four rotor commands','Each: 1.3935627 and 1.6335627 (model input units).'],
['Roll, pitch, yaw','Each: -0.1 and +0.1 rad.'],
['Three angular rates','Each: -0.3 and +0.3 rad/s.'],
['Position and linear velocity','All fixed to zero in the grid.']], [180,327])
add('The count is 2<super>4</super> x 2<super>3</super> x 2<super>3</super> = 1,024. Nominal drag is zero, so these Jacobians are independent of linear velocity; this is not a hover-only controller. The permitted velocity box remains [-2, 2]<super>3</super> m/s. Construction and verification use the same grid settings.')
h('Reproduction outcome')
add('Both SDPs returned OPTIMAL. The repeated run took 265 seconds and reproduced objectives 1494.858118 and 38.564610. The uncalibrated observer bound was epsilon = 0.07442638619. Several tightened constraint margins were negative: successful offline optimization did not yield usable uncalibrated MPC constraint sets.')
add('The repository subsequently uses empirical calibration to reduce conservatism. That stage was not used in our bound comparisons.','small')

start(4,'Where the propagation equations come from','Subtract the observer from the plant to obtain estimation error. Subtract the nominal predictor from the observer to obtain tracking error.')
eq(r'\dot e=f_0(x,u)-f_0(\hat{x},u)-LCe+E_0D(a)w^0-LF\eta','6')
eq(r'\dot\delta=f_0(\hat{x},u)-f_0(z,u_{\rm nom})+LCe+LF\eta','7')
add('Process disturbance enters estimation error directly. It affects estimate-to-nominal tracking through LCe. Measurement noise also enters tracking directly through the observer correction. The plant-to-nominal error x-z contains both errors.')
h('First prototype: separate norm bounds')
eq(r'\dot r=-\rho_o r+b_{\rm other}+a_{\rm upper}b_{\rm acc}+b_{\rm noise}','8')
eq(r'\dot s=-\rho_c s+\gamma r+b_{\rm noise}','9')
add('Contraction removes existing error. The b terms bound metric-norm contributions: other process disturbances, acceleration disturbances, and observer-injected measurement noise. The scalar gamma bounds the gain from estimation error into tracking error.')
eq(r'b_{\rm noise}=\max_\eta\|TLF\eta\|_2,\qquad \gamma=\|TLC T^{-1}\|_2','10')
add('Separate maximizations followed by the triangle inequality made this first construction conservative. It reused the baseline metric and gains, but selected contraction rates from numerical audits with a margin.')
table(['First prototype comparison','Final estimation radius'],[
['Global forcing in the new norm formula','0.105718'],
['Local forcing in the same norm formula','0.097925'],
['Original uncalibrated SDP bound','0.074426']], [365,142])
note('Why we changed the derivation','Local information helped within the first prototype, but it did not beat the original SDP bound. We therefore returned to the original quadratic observer LMI instead of summing separate norm bounds.')

start(5,'Recover the original observer bound','This is the key bridge from the authors\' offline certificate to a dynamic squared estimation radius.')
add('Write X = (P-delta)<super>-1</super>, A<sub>o</sub> = A - LC, lambda = lambda<sub>epsilon</sub>, W = W_bar and H = H1_bar. The metric is constant, so dot-X is zero. Define the symmetric disturbance block:')
story.append(BlockMatrix(r'S(b)=',[[r'0',r'b'],[r'b^{\mathrm{T}}',r'0']]))
eq(r'S(E_0w^0)\preceq W,\qquad S(-LF\eta)\preceq H','11')
eq(r'W+H+\operatorname{diag}(XA_o^{\mathrm{T}}+A_oX+\lambda X,-q_0)\preceq0','12')
eq(r'q_0=\lambda\epsilon^2','13')
add('Adding these inequalities cancels the auxiliary matrices. Evaluate the resulting matrix quadratic form at [P-delta e; 1]. With V = e<super>T</super>P-delta e, this gives:')
eq(r'\dot V\leq-\lambda V+q_0','14')
add('For nonlinear dynamics, A must cover the Jacobians along the segment from the estimate to the true state. The algebra is exact if the underlying matrix inequalities hold on that domain; our present checks are numerical and finite.')
h('Propagate the bound rather than fixing it')
eq(r'\dot R=-\lambda R+q_0,\quad R(0)\geq V(0),\quad r=\sqrt{R}','15')
eq(r'R(t)=\epsilon^2+[R(0)-\epsilon^2]e^{-\lambda t}','16')
note('Exact recovery of the baseline formula','If R(0) = epsilon<super>2</super>, then r(t) = epsilon at every time. Numerically, sqrt(q<sub>0</sub>/lambda) = 0.07442638619, exactly matching the saved baseline. A zero initial radius is valid only when initial estimation error is zero.')

start(6,'Reduce the process envelope locally','We keep the metric, gains and noise envelope. Only the process-envelope allowance changes with the prescribed uncertainty scale.')
story.append(BlockMatrix(r'W=',[[r'Q',r'h'],[r'h^{\mathrm{T}}',r'c']]))
add('In the saved design Q is positive definite. The Schur complement converts the process LMI into a quadratic bound:')
eq(r'S(E_0D(a)w^0)\preceq W\ \Longleftrightarrow\ (E_0D(a)w^0-h)^{\mathrm{T}}Q^{-1}(E_0D(a)w^0-h)\leq c','17')
eq(r'g(a)=\max_{w^0\in\operatorname{vert}(\mathcal{W}_0)}(E_0D(a)w^0-h)^{\mathrm{T}}Q^{-1}(E_0D(a)w^0-h)','18')
add('The quadratic is convex in disturbance, so its maximum over the box occurs at a vertex. Each vertex expression is convex in a; therefore g(a) is convex. Because the centered sets are nested, g(beta) is no larger than g(1).')
eq(r'g(a)\leq g(1)-\frac{1-a}{1-\beta}\,[g(1)-g(\beta)]','19')
add('We use 99% of this available reduction, retaining a numerical cushion:')
eq(r'\Delta=0.99[g(1)-g(\beta)],\qquad c_{\rm local}(a)=c-\Delta\frac{1-a}{1-\beta}','20')
eq(r'q(a)=q_0-\Delta\frac{1-a}{1-\beta}','21')
add('Keep Q and h fixed and replace c by c<sub>local</sub>. Decreasing the process matrix\'s bottom-right entry and decreasing q by the same amount leaves equation (12) unchanged. The new process envelope still contains the local disturbance set, conditional on the original inequalities.')
note('Why this avoids the shared-envelope bottleneck','There is now a family W<sub>local</sub>(a), rather than one worst-case process allowance at every state. At a = 1 it recovers the original W and q<sub>0</sub>. No new SDP was needed for this particular nested-box construction.')

start(7,'Use a bound on the unknown true speed','The disturbance depends on true velocity, but the true state is not known. Both error tubes must therefore enter the local uncertainty calculation.')
eq(r'a(v)=\beta+(1-\beta)\frac{\|v\|_2^2}{v_{\max}^2},\quad\beta=0.25,\quad v_{\max}=\sqrt{12}\ {\rm m/s}','22')
add('Only acceleration-disturbance amplitudes use this scale: 25% of the original maximum at zero speed and 100% at the domain\'s maximum speed. Other process disturbances and measurement noise remain at their baseline bounds.')
eq(r'c_v=\|S_vT^{-1}\|_2,\qquad \|v_{\rm true}\|_2\leq\|v_z\|_2+c_v(\sqrt{R}+s)','23')
eq(r'v_{\rm upper}=\min\{v_{\max},\ \|v_z\|_2+c_v(\sqrt{R}+s)\}','24')
eq(r'a_{\rm upper}=\beta+(1-\beta)(v_{\rm upper}/v_{\max})^2','25')
h('The improved coupled propagation')
eq(r'\dot R=-\lambda R+q(a_{\rm upper}),\qquad r=\sqrt{R}','26')
eq(r'\dot s=-\rho_c s+\gamma\sqrt{R}+b_{\rm noise}','27')
add('R affects s through the observer correction. Conversely, both radii affect the upper bound on true speed, which sets the process forcing in R. A low-speed region permits contraction over time; it does not justify an instantaneous reduction of estimation error.')
note('Assumptions required for the comparison','Initial radii must contain initial errors. States, estimates, nominal states and relevant inputs must remain in the domain supporting the LMIs. The speed cap is valid only under that domain assumption. These propagation equations do not themselves prove domain preservation.')
add('The scalar forcing is nondecreasing with uncertainty scale. With the same initial radii and valid derivative inequalities, the local coupled comparison can be ordered below the corresponding global-forcing comparison. Strict improvement is trajectory-dependent, not universal.','small')

# Replot saved data for a consistent, readable figure.
run=ROOT/'research/state_dependent_bounds/results/lmi_20260910_222354'
with (run/'lmi_radius_comparison.csv').open(newline='') as f: rows=list(csv.DictReader(f))
data={k:[float(r[k]) for r in rows] for k in rows[0]}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(3,1,figsize=(7.05,4.4),sharex=True,layout='constrained')
axs[0].plot(data['time'],data['nominal_speed'],color='#596E80',lw=1.8); axs[0].set_ylabel('Nominal speed\n(m/s)')
axs[1].plot(data['time'],data['local_r'],color='#087F8C',lw=2,label='Local forcing'); axs[1].plot(data['time'],data['original_epsilon'],color='#D17A28',lw=1.7,ls='--',label='Original/global'); axs[1].set_ylabel('Estimation\nradius'); axs[1].legend(loc='lower right',frameon=False,fontsize=8)
axs[2].plot(data['time'],data['local_s'],color='#087F8C',lw=2); axs[2].plot(data['time'],data['global_s'],color='#D17A28',lw=1.7,ls='--'); axs[2].set_ylabel('Tracking\nradius'); axs[2].set_xlabel('Time (s)')
for ax in axs: ax.grid(alpha=.18); ax.set_xlim(0,32)
plot=WORK/'comparison.png'; fig.savefig(plot,dpi=220); plt.close(fig)
start(8,'What the numerical comparison shows','The improved LMI-based construction uses the same metric, gains, noise bounds and initial radii in both comparisons: R(0) = epsilon squared and s(0) = 0.')
story.append(Image(str(plot),width=WIDTH,height=WIDTH*4.4/7.05)); story.append(Spacer(1,9))
add('Figure 1. Prescribed nominal-speed schedule and resulting comparison-ODE radii. This is not a simulated quadrotor trajectory and need not be dynamically feasible.','small')
table(['At t = 32 s','Original/global','Local forcing'],[
['Estimation radius','0.07442638619','0.06945456129'],
['Tracking radius','0.392384','0.367532']], [213,147,147])
note('Interpretation','The final estimation radius is 6.68% smaller in this example. This improves on the original uncalibrated bound within the baseline-equivalent propagation comparison. It does not establish lower actual estimation error, universal improvement, or closed-loop MPC safety.')
add('Parameters: lambda approximately 0.497029; q<sub>0</sub> = 0.002753187883; Delta approximately 0.000441487. The first prototype\'s larger radius is not used as the baseline here.','small')

start(9,'Evidence, assumptions and unfinished work','Distinguish an algebraic implication, a numerical check and a closed-loop robustness claim.')
table(['Evidence completed','What it establishes'],[
['Baseline recovery','The global squared-radius formula reproduces the original epsilon algebraically and numerically.'],
['Local-envelope derivation','Convexity and the Schur complement justify the local reduction, conditional on the original matrix inequalities.'],
['Numerical LMI audits','Original process/noise vertices and 1,024 dynamics points checked against the saved tolerance.'],
['Local tests','All 4,096 process vertices at 31 scales; 1,968 admissible nonlinear error configurations; scalar ODE and trajectory-ordering tests.']], [155,352])
add('The maximum original matrix residuals were approximately +5.6 x 10<super>-8</super>, within the code\'s tolerance of 10<super>-6</super>. They were not strictly negative. A formal certificate must also address numerical residuals, not merely accept the solver status.')
h('The required next steps')
table(['Priority','Work still required'],[
['1. Domain coverage','Bound nonlinear Jacobians between operating points and verify observer/tracking inequalities over that enclosure.'],
['2. Numerical rigor','Treat residuals and conditioning explicitly; introduce verified margins or certified numerical bounds.'],
['3. MPC construction','Adapt initialization, constraint tightening, terminal conditions and recursive-feasibility arguments.'],
['4. Closed-loop evaluation','Use our simulator to check actual errors, constraints and tube containment through low/high-uncertainty transitions.']], [125,382])
note('Uncertainty assumptions are separate from controller guarantees','Our local sets are prescribed subsets of the original UQ envelope. Containment in a global set does not prove real disturbances belong to the smaller local sets. Applying the claim to Gazebo or hardware requires additional model/data validation.')

start(10,'Code map and reproducibility','Use the WSL checkout: it preserves the distinct filenames compute_L.m and compute_l.m. Windows MATLAB, CasADi, YALMIP and MOSEK were used for the baseline.')
h('Original offline design: read in this order')
table(['File in mpc-sdp','Role'],[
['main.m','Coordinates both SDPs and saves offline_design.mat. It overwrites that output.'],
['FalconModelT.m','Dynamics, Jacobians, constraints, grid and uncertainty vertices.'],
['compute_Pdelta_Kdelta_cs_co.m','Main robust-design SDP and post-solve checks.'],
['compute_check_epsilon_RPI_LMIs.m','Observer envelope inequalities (11)-(12).'],
['compute_w_o_bar_c.m','Tracking growth gamma x epsilon + noise.'],
['compute_P.m','Terminal-cost SDP; separate from the error metric.']], [235,272])
h('Our research files')
table(['File','Role'],[
['run_lmi_radius_recovery.m','Current experiment, including local lmi_rhs and validation checks.'],
['LMI_RADIUS_DERIVATION.md','Detailed derivation and scope of numerical evidence.'],
['run_dynamic_bound_demo.m','Earlier, more conservative norm-based prototype.'],
['bound_rhs.m; check_error_dynamics.m','RHS and nonlinear tests for the earlier prototype only.']], [235,272])
add('<b>Project root:</b> /home/satyam/ROHMPC-State-Dependent-Uncertainty<br/><b>Original MATLAB folder:</b> src/catkin_ws/src/mpc/mpc_solver/scripts/include/offline_computations/mpc-sdp<br/><b>Our code:</b> research/state_dependent_bounds<br/><b>Saved baseline:</b> work/offline-sanity-20260910/offline_design.mat<br/><b>Improved-run evidence:</b> research/state_dependent_bounds/results/lmi_20260910_222354','small')
h('Sources and version anchors')
add('Benders et al., <i>From Data to Safe Mobile Robot Navigation: An Efficient and Modular Robust MPC Design Pipeline</i>, Sections II-IV, especially (7e), (12)-(13). <link href="https://arxiv.org/abs/2508.07045">arxiv.org/abs/2508.07045</link><br/>Public code: <link href="https://github.com/dbenders1/rohmpc">github.com/dbenders1/rohmpc</link>. Pinned mpc-sdp revision: ca4ba0c7b6f27bd2867b39a5bf5d666887901ecf. Input-data submodule revision: 5f3397db1e170496cc8c2b72977b8580292426c0. Results and derivations in this report are from our local runs; they are not additional claims made by the original paper.','small')

doc=SimpleDocTemplate(str(PDF),pagesize=(595.28,841.89),rightMargin=44,leftMargin=44,topMargin=43,bottomMargin=52,title='ROHMPC: Offline Certificates and State-Dependent Error Bounds',author='Prepared for Satyam Awasthi',subject='Baseline reproduction, derivation, code guide and validation status')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
reader=PdfReader(str(PDF)); counts=[len(pg.extract_text() or '') for pg in reader.pages]
print(json.dumps({'pdf':str(PDF),'pages':len(reader.pages),'text_characters_per_page':counts,'bytes':PDF.stat().st_size}))
assert len(reader.pages)==10, f'Expected 10 deliberately composed pages, got {len(reader.pages)}'
assert all(x>600 for x in counts),'Unexpected near-empty page'