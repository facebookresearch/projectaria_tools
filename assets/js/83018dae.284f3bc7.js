"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[9163],{15680:(e,t,a)=>{a.r(t),a.d(t,{MDXContext:()=>l,MDXProvider:()=>c,mdx:()=>h,useMDXComponents:()=>s,withMDXComponents:()=>p});var n=a(96540);function r(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function i(){return i=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var a=arguments[t];for(var n in a)Object.prototype.hasOwnProperty.call(a,n)&&(e[n]=a[n])}return e},i.apply(this,arguments)}function m(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,n)}return a}function d(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?m(Object(a),!0).forEach((function(t){r(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):m(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function o(e,t){if(null==e)return{};var a,n,r=function(e,t){if(null==e)return{};var a,n,r={},i=Object.keys(e);for(n=0;n<i.length;n++)a=i[n],t.indexOf(a)>=0||(r[a]=e[a]);return r}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(n=0;n<i.length;n++)a=i[n],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(r[a]=e[a])}return r}var l=n.createContext({}),p=function(e){return function(t){var a=s(t.components);return n.createElement(e,i({},t,{components:a}))}},s=function(e){var t=n.useContext(l),a=t;return e&&(a="function"==typeof e?e(t):d(d({},t),e)),a},c=function(e){var t=s(e.components);return n.createElement(l.Provider,{value:t},e.children)},u="mdxType",f={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},g=n.forwardRef((function(e,t){var a=e.components,r=e.mdxType,i=e.originalType,m=e.parentName,l=o(e,["components","mdxType","originalType","parentName"]),p=s(a),c=r,u=p["".concat(m,".").concat(c)]||p[c]||f[c]||i;return a?n.createElement(u,d(d({ref:t},l),{},{components:a})):n.createElement(u,d({ref:t},l))}));function h(e,t){var a=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var i=a.length,m=new Array(i);m[0]=g;var d={};for(var o in t)hasOwnProperty.call(t,o)&&(d[o]=t[o]);d.originalType=e,d[u]="string"==typeof e?e:r,m[1]=d;for(var l=2;l<i;l++)m[l]=a[l];return n.createElement.apply(null,m)}return n.createElement.apply(null,a)}g.displayName="MDXCreateElement"},22013:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>o,contentTitle:()=>m,default:()=>c,frontMatter:()=>i,metadata:()=>d,toc:()=>l});var n=a(58168),r=(a(96540),a(15680));const i={sidebar_position:50,title:"Hand Tracking"},m="MPS Outputs - Hand Tracking",d={unversionedId:"data_formats/mps/hand_tracking/hand_tracking",id:"data_formats/mps/hand_tracking/hand_tracking",title:"Hand Tracking",description:"Project Aria's Machine Perception Services (MPS) uses Project Aria's SLAM (mono scene) camera images to estimate the hand movement of the wearer. Hand Tracking outputs are available for all recordings made with SLAM cameras and can be requested using the MPS CLI or Desktop Companion app.",source:"@site/docs/data_formats/mps/hand_tracking/hand_tracking.mdx",sourceDirName:"data_formats/mps/hand_tracking",slug:"/data_formats/mps/hand_tracking/",permalink:"/projectaria_tools/docs/data_formats/mps/hand_tracking/",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_formats/mps/hand_tracking/hand_tracking.mdx",tags:[],version:"current",sidebarPosition:50,frontMatter:{sidebar_position:50,title:"Hand Tracking"},sidebar:"tutorialSidebar",previous:{title:"Eye Gaze",permalink:"/projectaria_tools/docs/data_formats/mps/mps_eye_gaze"},next:{title:"2D Image Coordinate System Conventions",permalink:"/projectaria_tools/docs/data_formats/coordinate_convention/2d_image_coordinate_system_convention"}},o={},l=[{value:"Hand Tracking Outputs",id:"hand-tracking-outputs",level:2},{value:"wrist_and_palm_poses.csv",id:"wrist_and_palm_posescsv",level:3},{value:"summary.json",id:"summaryjson",level:2}],p={toc:l},s="wrapper";function c(e){let{components:t,...a}=e;return(0,r.mdx)(s,(0,n.A)({},p,a,{components:t,mdxType:"MDXLayout"}),(0,r.mdx)("h1",{id:"mps-outputs---hand-tracking"},"MPS Outputs - Hand Tracking"),(0,r.mdx)("p",null,"Project Aria's ",(0,r.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/"},"Machine Perception Services (MPS)")," uses Project Aria's SLAM (mono scene) camera images to estimate the hand movement of the wearer. Hand Tracking outputs are available for all recordings made with SLAM cameras and can be requested using the MPS CLI or Desktop Companion app."),(0,r.mdx)("p",null,"Go to ",(0,r.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/core_code_snippets/mps"},"MPS Code Snippets")," for how to:"),(0,r.mdx)("ul",null,(0,r.mdx)("li",{parentName:"ul"},"Load the wrist and palm positions"),(0,r.mdx)("li",{parentName:"ul"},"Transform them to the world coordinate frame"),(0,r.mdx)("li",{parentName:"ul"},"Project them onto the SLAM or RGB images")),(0,r.mdx)("p",null,"Hand Tracking data can be visualized in ",(0,r.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_python"},"Python")," or ",(0,r.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_cpp"},"C++"),"."),(0,r.mdx)("h2",{id:"hand-tracking-outputs"},"Hand Tracking Outputs"),(0,r.mdx)("p",null,"We currently provide Wrist and Palm Tracking data. The file outputs are:"),(0,r.mdx)("ul",null,(0,r.mdx)("li",{parentName:"ul"},(0,r.mdx)("inlineCode",{parentName:"li"},"wrist_and_palm_poses.csv")," - the coordinates of the wrist and palm positions in the device frame"),(0,r.mdx)("li",{parentName:"ul"},(0,r.mdx)("inlineCode",{parentName:"li"},"summary.json")," - high-level report on MPS wrist and palm tracking")),(0,r.mdx)("h3",{id:"wrist_and_palm_posescsv"},"wrist_and_palm_poses.csv"),(0,r.mdx)("p",null,(0,r.mdx)("inlineCode",{parentName:"p"},"wrist_and_palm_poses.csv")," contains the following fields:"),(0,r.mdx)("table",null,(0,r.mdx)("thead",{parentName:"table"},(0,r.mdx)("tr",{parentName:"thead"},(0,r.mdx)("th",{parentName:"tr",align:null},(0,r.mdx)("strong",{parentName:"th"},"Column")),(0,r.mdx)("th",{parentName:"tr",align:null},(0,r.mdx)("strong",{parentName:"th"},"Type")),(0,r.mdx)("th",{parentName:"tr",align:null},(0,r.mdx)("strong",{parentName:"th"},"Description")))),(0,r.mdx)("tbody",{parentName:"table"},(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tracking_timestamp_us"),(0,r.mdx)("td",{parentName:"tr",align:null},"int"),(0,r.mdx)("td",{parentName:"tr",align:null},"Timestamp, in microseconds, of the SLAM camera frame in device time domain. This is the same time domain in which the MPS ",(0,r.mdx)("a",{parentName:"td",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_trajectory#open-loop-trajectory"},"trajectory outputs")," are reported, so these timestamps can be directly used to infer the device pose from the MPS trajectory output.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"left_tracking_confidence"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"A value between 0.0 and 1.0 indicating the confidence in the reported left wrist and palm positions. A value of -1.0 indicates that the left wrist and palm tracking data is missing for the frame, and the coordinates left",(0,r.mdx)("em",{parentName:"td"},"wrist_device"),(0,r.mdx)("em",{parentName:"td"}," and left",(0,r.mdx)("em",{parentName:"em"},"palm_device"))," should not be used.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tx_left_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"X-coordinate of the left wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"ty_left_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Y-coordinate of the left wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tz_left_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Z-coordinate of the left wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tx_left_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"X-coordinate of the left palm position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"ty_left_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Y-coordinate of the left palm position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tz_left_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Z-coordinate of the left palm position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"right_tracking_confidence"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"A value between 0.0 and 1.0 indicating the confidence in the reported right wrist and palm positions. A value of -1.0 indicates that the right wrist and palm tracking data is missing for the frame, and the coordinates right",(0,r.mdx)("em",{parentName:"td"},"wrist_device"),(0,r.mdx)("em",{parentName:"td"}," and right",(0,r.mdx)("em",{parentName:"em"},"palm_device"))," should not be used.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tx_right_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"X-coordinate of the right wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"ty_right_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Y-coordinate of the right wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tz_right_wrist_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Z-coordinate of the right wrist position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tx_right_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"X-coordinate of the right palm position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"ty_right_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Y-coordinate of the right palm position given in the device frame in meters.")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"tz_right_palm_device"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Z-coordinate of the right palm position given in the device frame in meters.")))),(0,r.mdx)("p",null,"The wrist and palm poses are given in the device frame in meters."),(0,r.mdx)("h2",{id:"summaryjson"},"summary.json"),(0,r.mdx)("p",null,(0,r.mdx)("inlineCode",{parentName:"p"},"summary.json")," contains the ",(0,r.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/mps_summary#operator-summary"},"Operator Summary"),", listed in MPS Basics, as well as the following fields:"),(0,r.mdx)("table",null,(0,r.mdx)("thead",{parentName:"table"},(0,r.mdx)("tr",{parentName:"thead"},(0,r.mdx)("th",{parentName:"tr",align:null},"Field"),(0,r.mdx)("th",{parentName:"tr",align:null},"Type"),(0,r.mdx)("th",{parentName:"tr",align:null},"Description"))),(0,r.mdx)("tbody",{parentName:"table"},(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"mean_confidence"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Average ",(0,r.mdx)("inlineCode",{parentName:"td"},"left_tracking_confidence")," and ",(0,r.mdx)("inlineCode",{parentName:"td"},"right_tracking_confidence")," value for frames with valid results")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"total_frames"),(0,r.mdx)("td",{parentName:"tr",align:null},"int"),(0,r.mdx)("td",{parentName:"tr",align:null},"Total number of frames")),(0,r.mdx)("tr",{parentName:"tbody"},(0,r.mdx)("td",{parentName:"tr",align:null},"valid_frame_fraction"),(0,r.mdx)("td",{parentName:"tr",align:null},"float"),(0,r.mdx)("td",{parentName:"tr",align:null},"Fraction of frames that have reported a valid tracking result")))))}c.isMDXComponent=!0}}]);