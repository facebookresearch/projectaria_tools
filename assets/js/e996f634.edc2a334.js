"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[9441],{15680:(e,t,a)=>{a.r(t),a.d(t,{MDXContext:()=>l,MDXProvider:()=>u,mdx:()=>_,useMDXComponents:()=>d,withMDXComponents:()=>p});var r=a(96540);function n(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function i(){return i=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var a=arguments[t];for(var r in a)Object.prototype.hasOwnProperty.call(a,r)&&(e[r]=a[r])}return e},i.apply(this,arguments)}function o(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,r)}return a}function s(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?o(Object(a),!0).forEach((function(t){n(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):o(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function m(e,t){if(null==e)return{};var a,r,n=function(e,t){if(null==e)return{};var a,r,n={},i=Object.keys(e);for(r=0;r<i.length;r++)a=i[r],t.indexOf(a)>=0||(n[a]=e[a]);return n}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(r=0;r<i.length;r++)a=i[r],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(n[a]=e[a])}return n}var l=r.createContext({}),p=function(e){return function(t){var a=d(t.components);return r.createElement(e,i({},t,{components:a}))}},d=function(e){var t=r.useContext(l),a=t;return e&&(a="function"==typeof e?e(t):s(s({},t),e)),a},u=function(e){var t=d(e.components);return r.createElement(l.Provider,{value:t},e.children)},c="mdxType",h={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},f=r.forwardRef((function(e,t){var a=e.components,n=e.mdxType,i=e.originalType,o=e.parentName,l=m(e,["components","mdxType","originalType","parentName"]),p=d(a),u=n,c=p["".concat(o,".").concat(u)]||p[u]||h[u]||i;return a?r.createElement(c,s(s({ref:t},l),{},{components:a})):r.createElement(c,s({ref:t},l))}));function _(e,t){var a=arguments,n=t&&t.mdxType;if("string"==typeof e||n){var i=a.length,o=new Array(i);o[0]=f;var s={};for(var m in t)hasOwnProperty.call(t,m)&&(s[m]=t[m]);s.originalType=e,s[c]="string"==typeof e?e:n,o[1]=s;for(var l=2;l<i;l++)o[l]=a[l];return r.createElement.apply(null,o)}return r.createElement.apply(null,a)}f.displayName="MDXCreateElement"},13431:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>m,contentTitle:()=>o,default:()=>u,frontMatter:()=>i,metadata:()=>s,toc:()=>l});var r=a(58168),n=(a(96540),a(15680));const i={sidebar_position:10,title:"Basics"},o=void 0,s={unversionedId:"data_formats/mps/mps_summary",id:"data_formats/mps/mps_summary",title:"Basics",description:"This page provides an overview of how Project Aria Machine Perception Services (MPS) output files are formatted.",source:"@site/docs/data_formats/mps/mps_summary.mdx",sourceDirName:"data_formats/mps",slug:"/data_formats/mps/mps_summary",permalink:"/projectaria_tools/docs/data_formats/mps/mps_summary",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_formats/mps/mps_summary.mdx",tags:[],version:"current",sidebarPosition:10,frontMatter:{sidebar_position:10,title:"Basics"},sidebar:"tutorialSidebar",previous:{title:"Timestamp Definitions",permalink:"/projectaria_tools/docs/data_formats/aria_vrs/timestamps_in_aria_vrs"},next:{title:"SLAM MPS Outputs",permalink:"/projectaria_tools/docs/data_formats/mps/slam/"}},m={},l=[{value:"MPS File Structure",id:"mps-file-structure",level:2},{value:"Common terminologies",id:"common-terminologies",level:2},{value:"graph_uid",id:"graph_uid",level:3},{value:"tracking_timestamp_us",id:"tracking_timestamp_us",level:3},{value:"utc_timestamp_ns",id:"utc_timestamp_ns",level:3},{value:"Operator summary",id:"operator-summary",level:2}],p={toc:l},d="wrapper";function u(e){let{components:t,...a}=e;return(0,n.mdx)(d,(0,r.A)({},p,a,{components:t,mdxType:"MDXLayout"}),(0,n.mdx)("p",null,"This page provides an overview of how Project Aria ",(0,n.mdx)("a",{parentName:"p",href:"/docs/ARK/mps"},"Machine Perception Services (MPS)")," output files are formatted."),(0,n.mdx)("p",null,"MPS provides derived data generated from Aria raw data (VRS files)."),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},"Go to ",(0,n.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/ARK/mps/"},"MPS Overview")," for an introduction to MPS"),(0,n.mdx)("li",{parentName:"ul"},"Go to ",(0,n.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli"},"Request MPS")," for how to request MPS on Aria raw data, if you have access to the ",(0,n.mdx)("a",{parentName:"li",href:"https://www.projectaria.com/research-kit/"},"Aria Research Kit"))),(0,n.mdx)("h2",{id:"mps-file-structure"},"MPS File Structure"),(0,n.mdx)("p",null,"MPS outputs use the following structure, in this example recording1.vrs was used to generate MPS."),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre"},"    \u2514\u2500\u2500 Example folder\n        \u251c\u2500\u2500 mps_recording1_vrs\n        \u2502   \u251c\u2500\u2500 eye_gaze\n        \u2502   \u2502   \u251c\u2500\u2500 general_eye_gaze.csv\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 slam\n        \u2502   \u2502   \u251c\u2500\u2500 closed_loop_trajectory.csv\n        \u2502   \u2502   \u251c\u2500\u2500 online_calibration.jsonl\n        \u2502   \u2502   \u251c\u2500\u2500 open_loop_trajectory.csv\n        \u2502   \u2502   \u251c\u2500\u2500 semidense_observations.csv.gz\n        \u2502   \u2502   \u251c\u2500\u2500 semidense_points.csv.gz\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 hand_tracking\n        \u2502   \u2502   \u251c\u2500\u2500 wrist_and_palm_poses.csv\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 vrs_health_check.json\n        \u2502   \u2514\u2500\u2500 vrs_health_check_slam.json\n        \u2514\u2500\u2500 recording1.vrs\n")),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},"mps_","[name of VRS file]","_vrs",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"Sibling directory where all the intermediate data and MPS output is saved"))),(0,n.mdx)("li",{parentName:"ul"},"vrs_health_check.json",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"Output of the health check performed on your computer before data is uploaded"),(0,n.mdx)("li",{parentName:"ul"},"Contains information about data drops in all the sensor streams"))),(0,n.mdx)("li",{parentName:"ul"},"vrs_health_check_slam.json",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"Summary of SLAM specific checks on the VRS files"),(0,n.mdx)("li",{parentName:"ul"},"If the health check fails it will contain details about which health checks failed"))),(0,n.mdx)("li",{parentName:"ul"},"slam folder",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"Contains outputs after running SLAM (Trajectory and Semi-Dense Point Cloud data)"))),(0,n.mdx)("li",{parentName:"ul"},"eye_gaze folder",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"Contains eye gaze outputs")))),(0,n.mdx)("h2",{id:"common-terminologies"},"Common terminologies"),(0,n.mdx)("h3",{id:"graph_uid"},"graph_uid"),(0,n.mdx)("p",null,(0,n.mdx)("inlineCode",{parentName:"p"},"graph_uid")," is a unique identifier for the world coordinate frame. For all the 3D geometric instances like pose and points in the world frames (having _world in the suffix), when they have the same graph_uid, they are in the same coordinate frame."),(0,n.mdx)("p",null,"For simulation (such as ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/open_datasets/aria_synthetic_environments_dataset/ase_getting_started"},"Aria Synthetic Environments"),") and ",(0,n.mdx)("a",{parentName:"p",href:"/docs/open_datasets/aria_digital_twin_dataset"},"Aria Digital Twin(ADT)")," datasets we use the same random value for one space, e.g. the same graph_uid for one ADT/simulation space."),(0,n.mdx)("h3",{id:"tracking_timestamp_us"},"tracking_timestamp_us"),(0,n.mdx)("p",null,(0,n.mdx)("inlineCode",{parentName:"p"},"tracking_timestamp_us"),"'s values are shaped by whether it is real world or synthetic data."),(0,n.mdx)("p",null,"For real world data, ",(0,n.mdx)("inlineCode",{parentName:"p"},"tracking_timestamp_us")," provides the Device timestamps from your Aria glasses. Go to ",(0,n.mdx)("a",{parentName:"p",href:"/docs/data_formats/aria_vrs/timestamps_in_aria_vrs"},"Timestamps in Aria VRS")," for a definition of the device timestamps."),(0,n.mdx)("p",null,"In simulation datasets, this will be the timestamp in the simulator."),(0,n.mdx)("p",null,"In ",(0,n.mdx)("inlineCode",{parentName:"p"},"tracking_timestamp_us")),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},"This clock has arbitrary starting points, which are not synchronized between recording sessions or devices."),(0,n.mdx)("li",{parentName:"ul"},"This clock is strictly monotonic, has stable clock speed, and is accurate in duration",(0,n.mdx)("ul",{parentName:"li"},(0,n.mdx)("li",{parentName:"ul"},"If you want to compute the time duration between two timestamps (especially when touching dynamics, e.g. integrating acceleration to velocity over time), you should use this timestamp.")))),(0,n.mdx)("h3",{id:"utc_timestamp_ns"},"utc_timestamp_ns"),(0,n.mdx)("p",null,(0,n.mdx)("inlineCode",{parentName:"p"},"utc_timestamp_ns")," is the timestamp from Aria real-time clock (RTC). This time is synchronized to the cell phone time via the Aria Mobile Companion app to get UTC time at the beginning of the recording which is a rough estimate of the external standard clock."),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},"This clock is not available in the simulation datasets."),(0,n.mdx)("li",{parentName:"ul"},"This clock provides rough synchronization between sessions and devices."),(0,n.mdx)("li",{parentName:"ul"},"This clock is not guaranteed to be monotonic, or have stable clock speed, due to synchronization with NTP. So do not compute duration between two UTC timestamps.")),(0,n.mdx)("h2",{id:"operator-summary"},"Operator summary"),(0,n.mdx)("p",null,"The operator summary includes individual operator\u2019s status and whether the operation was successful. There are three possible status flags:"),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},(0,n.mdx)("strong",{parentName:"li"},"SUCCESS"),": the operator successfully finished, without known issues."),(0,n.mdx)("li",{parentName:"ul"},(0,n.mdx)("strong",{parentName:"li"},"WARNING"),": The operator finished, but internally it detected problem(s) that may affect results quality. The operator still outputs the results, but we don\u2019t have confidence in the quality of the results, so consume the results with caution."),(0,n.mdx)("li",{parentName:"ul"},(0,n.mdx)("strong",{parentName:"li"},"ERROR"),": the operator did not finish, finished with major error, or the quality of the results are too bad to be consumed. Results may or may not be generated, and any results should not be consumed.")),(0,n.mdx)("p",null,"The summary also provides information about processes as well as any warning or error messages available"),(0,n.mdx)("p",null,"Summary JSON output example:"),(0,n.mdx)("pre",null,(0,n.mdx)("code",{parentName:"pre",className:"language-json"},'\n    "SLAM": {\n        "status": "SUCCESS",\n        "info": [\n            "Recording total time: 1104.00s; Trajectory total length: 155.42m",\n            "Total Vision Translational Correction (mm): p50: 0.048; p99: 0.451",\n            "Rotational Correction (deg): p50: 0.001; p99: 0.007"\n        ],\n        "warnings": [],\n        "errors": []\n    },\n...\n')))}u.isMDXComponent=!0}}]);