"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[3199],{15680:(e,t,r)=>{r.r(t),r.d(t,{MDXContext:()=>p,MDXProvider:()=>m,mdx:()=>v,useMDXComponents:()=>d,withMDXComponents:()=>l});var a=r(96540);function o(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}function i(){return i=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var a in r)Object.prototype.hasOwnProperty.call(r,a)&&(e[a]=r[a])}return e},i.apply(this,arguments)}function n(e,t){var r=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),r.push.apply(r,a)}return r}function s(e){for(var t=1;t<arguments.length;t++){var r=null!=arguments[t]?arguments[t]:{};t%2?n(Object(r),!0).forEach((function(t){o(e,t,r[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(r)):n(Object(r)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(r,t))}))}return e}function c(e,t){if(null==e)return{};var r,a,o=function(e,t){if(null==e)return{};var r,a,o={},i=Object.keys(e);for(a=0;a<i.length;a++)r=i[a],t.indexOf(r)>=0||(o[r]=e[r]);return o}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(a=0;a<i.length;a++)r=i[a],t.indexOf(r)>=0||Object.prototype.propertyIsEnumerable.call(e,r)&&(o[r]=e[r])}return o}var p=a.createContext({}),l=function(e){return function(t){var r=d(t.components);return a.createElement(e,i({},t,{components:r}))}},d=function(e){var t=a.useContext(p),r=t;return e&&(r="function"==typeof e?e(t):s(s({},t),e)),r},m=function(e){var t=d(e.components);return a.createElement(p.Provider,{value:t},e.children)},u="mdxType",f={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},h=a.forwardRef((function(e,t){var r=e.components,o=e.mdxType,i=e.originalType,n=e.parentName,p=c(e,["components","mdxType","originalType","parentName"]),l=d(r),m=o,u=l["".concat(n,".").concat(m)]||l[m]||f[m]||i;return r?a.createElement(u,s(s({ref:t},p),{},{components:r})):a.createElement(u,s({ref:t},p))}));function v(e,t){var r=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var i=r.length,n=new Array(i);n[0]=h;var s={};for(var c in t)hasOwnProperty.call(t,c)&&(s[c]=t[c]);s.originalType=e,s[u]="string"==typeof e?e:o,n[1]=s;for(var p=2;p<i;p++)n[p]=r[p];return a.createElement.apply(null,n)}return a.createElement.apply(null,r)}h.displayName="MDXCreateElement"},41699:(e,t,r)=>{r.r(t),r.d(t,{assets:()=>c,contentTitle:()=>n,default:()=>m,frontMatter:()=>i,metadata:()=>s,toc:()=>p});var a=r(58168),o=(r(96540),r(15680));const i={sidebar_position:10,title:"Machine Perception Services (MPS)"},n="Project Aria Machine Perception Services",s={unversionedId:"ARK/mps/mps",id:"ARK/mps/mps",title:"Machine Perception Services (MPS)",description:"To accelerate research with Project Aria, we provide several Spatial AI machine perception capabilities that help form the foundation for future Contextualized AI applications and analysis of egocentric data. These capabilities are powered by a set of proprietary machine perception algorithms, designed for Project Aria glasses, that provide superior accuracy and robustness on Aria data compared to off-the-shelf open source algorithms.",source:"@site/docs/ARK/mps/mps.mdx",sourceDirName:"ARK/mps",slug:"/ARK/mps/",permalink:"/projectaria_tools/docs/ARK/mps/",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/ARK/mps/mps.mdx",tags:[],version:"current",sidebarPosition:10,frontMatter:{sidebar_position:10,title:"Machine Perception Services (MPS)"},sidebar:"tutorialSidebar",previous:{title:"SDK Troubleshooting & Known Issues",permalink:"/projectaria_tools/docs/ARK/sdk/sdk_troubleshooting"},next:{title:"MPS CLI (Recommended)",permalink:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli"}},c={},p=[{value:"Current MPS offerings",id:"current-mps-offerings",level:2},{value:"SLAM services",id:"slam-services",level:2},{value:"6DoF trajectory",id:"6dof-trajectory",level:3},{value:"Semi-dense point cloud",id:"semi-dense-point-cloud",level:3},{value:"Online sensor calibration",id:"online-sensor-calibration",level:3},{value:"Multi-SLAM",id:"multi-slam",level:3},{value:"Eye Gaze services",id:"eye-gaze-services",level:2},{value:"Hand Tracking",id:"hand-tracking",level:2},{value:"Wrist and Palm Tracking",id:"wrist-and-palm-tracking",level:3},{value:"About MPS Data Loader APIs",id:"about-mps-data-loader-apis",level:2},{value:"Questions &amp; Feedback",id:"questions--feedback",level:2}],l={toc:p},d="wrapper";function m(e){let{components:t,...r}=e;return(0,o.mdx)(d,(0,a.A)({},l,r,{components:t,mdxType:"MDXLayout"}),(0,o.mdx)("h1",{id:"project-aria-machine-perception-services"},"Project Aria Machine Perception Services"),(0,o.mdx)("p",null,"To accelerate research with Project Aria, we provide several Spatial AI machine perception capabilities that help form the foundation for future Contextualized AI applications and analysis of egocentric data. These capabilities are powered by a set of proprietary machine perception algorithms, designed for Project Aria glasses, that provide superior accuracy and robustness on Aria data compared to off-the-shelf open source algorithms."),(0,o.mdx)("p",null,"All Machine Perception Services (MPS) are offered as post-processing of VRS files via a cloud service. Use the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli"},"MPS CLI")," or the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/desktop_mps"},"Desktop App"),", to request derived data from any VRS file that contains necessary sensor data."),(0,o.mdx)("h2",{id:"current-mps-offerings"},"Current MPS offerings"),(0,o.mdx)("p",null,"The following MPS can be requested, as long as the data has been recorded with a compatible Recording Profile. The ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/glasses_manual/profile_guide"},"Recording Profile Guide")," provides a quick list of compatible sensor profile,s or go to ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/tech_spec/recording_profiles"},"Recording Profiles")," in Technical Specifications for more granular information about each profile."),(0,o.mdx)("p",null,"MPS offerings are grouped into SLAM, Eye Gaze and Hand Tracking services."),(0,o.mdx)("h2",{id:"slam-services"},"SLAM services"),(0,o.mdx)("p",null,"To get these outputs the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/tech_spec/recording_profiles"},"recording profile")," must have SLAM cameras + IMU enabled."),(0,o.mdx)("h3",{id:"6dof-trajectory"},"6DoF trajectory"),(0,o.mdx)("p",null,"MPS provides two types of high frequency (1kHz) trajectories:"),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_trajectory#open-loop-trajectory"},"Open loop trajectory")," - local odometry estimation from visual-inertial odometry (VIO)"),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_trajectory#closed-loop-trajectory"},"Closed loop trajectory")," - created via batch optimization, using multi-sensors' input (SLAM, IMU, barometer, Wi-Fi and GPS), fully optimized and provides poses in a consistent frame of reference.")),(0,o.mdx)("h3",{id:"semi-dense-point-cloud"},"Semi-dense point cloud"),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_pointcloud"},"Semi-dense point cloud")," data supports researchers who need static scene 3D reconstructions, reliable 2D images tracks or a representative visualization of the environment."),(0,o.mdx)("h3",{id:"online-sensor-calibration"},"Online sensor calibration"),(0,o.mdx)("p",null,"The ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_calibration"},"time-varying intrinsic and extrinsic calibrations")," of cameras and IMUs are estimated at the frequency of the SLAM (mono scene) cameras by our multi-sensor state estimation pipeline."),(0,o.mdx)("h3",{id:"multi-slam"},"Multi-SLAM"),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_multi_slam"},"Multi-SLAM")," can be requested on two or more recordings. It creates all of the above SLAM output, in a shared co-ordinate frame."),(0,o.mdx)("p",null,"Multi-SLAM can only be requested using the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli"},"MPS CLI SDK"),"."),(0,o.mdx)("h2",{id:"eye-gaze-services"},"Eye Gaze services"),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/mps_eye_gaze"},"Eye Gaze data")," is generated using Aria's Eye Tracking (ET) camera images to estimate the direction the user is looking. These outputs can be generated for any recording that had ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/tech_spec/recording_profiles"},"ET cameras enabled"),"."),(0,o.mdx)("p",null,"In March 2024, we updated our eye gaze model to support depth estimation. We do this by providing left and right eye gaze directions (yaw values) along with the depth at which these gaze directions intersect (translation values)."),(0,o.mdx)("p",null,"If you have made a recording with ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/eye_gaze_calibration"},"In-Session Eye Gaze Calibration"),", you will receive a second .csv file with calibrated eye gaze outputs."),(0,o.mdx)("h2",{id:"hand-tracking"},"Hand Tracking"),(0,o.mdx)("p",null,"To compute hand tracking outputs, the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/tech_spec/recording_profiles"},"recording profile")," must have SLAM cameras enabled."),(0,o.mdx)("h3",{id:"wrist-and-palm-tracking"},"Wrist and Palm Tracking"),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/hand_tracking/"},"Wrist and Palm Tracking data")," is created by using SLAM camera images to estimate the hand movement of the wearer. The wrist and palm poses are given in the device frame in meters."),(0,o.mdx)("h2",{id:"about-mps-data-loader-apis"},"About MPS Data Loader APIs"),(0,o.mdx)("p",null,"Please refer to our ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/core_code_snippets/mps#load-mps-output"},"MPS data loader APIs")," (C++ and Python support) to load the MPS outputs into your application. The ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_cpp#mps-static-scene-visualizer"},"visualization guide")," shows how to visualize all the MPS outputs."),(0,o.mdx)("h2",{id:"questions--feedback"},"Questions & Feedback"),(0,o.mdx)("p",null,"If you have feedback you'd like to provide, be it overall trends and experiences or where we can improve, we'd love to hear from you. Go to our ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/support"},"Support page")," for different ways to get in touch."))}m.isMDXComponent=!0}}]);