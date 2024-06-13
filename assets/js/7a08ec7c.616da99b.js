"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[3681],{15680:(e,t,a)=>{a.r(t),a.d(t,{MDXContext:()=>m,MDXProvider:()=>u,mdx:()=>g,useMDXComponents:()=>d,withMDXComponents:()=>p});var r=a(96540);function o(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function i(){return i=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var a=arguments[t];for(var r in a)Object.prototype.hasOwnProperty.call(a,r)&&(e[r]=a[r])}return e},i.apply(this,arguments)}function n(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,r)}return a}function s(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?n(Object(a),!0).forEach((function(t){o(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):n(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function l(e,t){if(null==e)return{};var a,r,o=function(e,t){if(null==e)return{};var a,r,o={},i=Object.keys(e);for(r=0;r<i.length;r++)a=i[r],t.indexOf(a)>=0||(o[a]=e[a]);return o}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(r=0;r<i.length;r++)a=i[r],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(o[a]=e[a])}return o}var m=r.createContext({}),p=function(e){return function(t){var a=d(t.components);return r.createElement(e,i({},t,{components:a}))}},d=function(e){var t=r.useContext(m),a=t;return e&&(a="function"==typeof e?e(t):s(s({},t),e)),a},u=function(e){var t=d(e.components);return r.createElement(m.Provider,{value:t},e.children)},c="mdxType",h={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},f=r.forwardRef((function(e,t){var a=e.components,o=e.mdxType,i=e.originalType,n=e.parentName,m=l(e,["components","mdxType","originalType","parentName"]),p=d(a),u=o,c=p["".concat(n,".").concat(u)]||p[u]||h[u]||i;return a?r.createElement(c,s(s({ref:t},m),{},{components:a})):r.createElement(c,s({ref:t},m))}));function g(e,t){var a=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var i=a.length,n=new Array(i);n[0]=f;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s[c]="string"==typeof e?e:o,n[1]=s;for(var m=2;m<i;m++)n[m]=a[m];return r.createElement.apply(null,n)}return r.createElement.apply(null,a)}f.displayName="MDXCreateElement"},1158:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>l,contentTitle:()=>n,default:()=>u,frontMatter:()=>i,metadata:()=>s,toc:()=>m});var r=a(58168),o=(a(96540),a(15680));const i={sidebar_position:20,title:"MPS CLI Getting Started"},n="Getting Started With the MPS CLI",s={unversionedId:"ARK/mps/request_mps/mps_cli_getting_started",id:"ARK/mps/request_mps/mps_cli_getting_started",title:"MPS CLI Getting Started",description:"Overview",source:"@site/docs/ARK/mps/request_mps/mps_cli_getting_started.mdx",sourceDirName:"ARK/mps/request_mps",slug:"/ARK/mps/request_mps/mps_cli_getting_started",permalink:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli_getting_started",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/ARK/mps/request_mps/mps_cli_getting_started.mdx",tags:[],version:"current",sidebarPosition:20,frontMatter:{sidebar_position:20,title:"MPS CLI Getting Started"},sidebar:"tutorialSidebar",previous:{title:"MPS CLI (Recommended)",permalink:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli"},next:{title:"MPS CLI Guide",permalink:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli_guide"}},l={},m=[{value:"Overview",id:"overview",level:2},{value:"Getting Started",id:"getting-started",level:2},{value:"Install Project Aria Tools",id:"install-project-aria-tools",level:3},{value:"Download the MPS CLI sample dataset",id:"download-the-mps-cli-sample-dataset",level:3},{value:"Request MPS for all VRS files in the \u201cExample\u201d directory and it\u2019s subdirectories:",id:"request-mps-for-all-vrs-files-in-the-example-directory-and-its-subdirectories",level:3},{value:"Exit the MPS CLI",id:"exit-the-mps-cli",level:3},{value:"Multi Sequence MPS Requests",id:"multi-sequence-mps-requests",level:3},{value:"Working with MPS data",id:"working-with-mps-data",level:2}],p={toc:m},d="wrapper";function u(e){let{components:t,...i}=e;return(0,o.mdx)(d,(0,r.A)({},p,i,{components:t,mdxType:"MDXLayout"}),(0,o.mdx)("h1",{id:"getting-started-with-the-mps-cli"},"Getting Started With the MPS CLI"),(0,o.mdx)("h2",{id:"overview"},"Overview"),(0,o.mdx)("p",null,"The Project Aria Machine Perceptions Services Command Line Interface (MPS CLI) is a command line tool used to request and receive ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/"},"Machine Perception Services"),". While you can use the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/desktop_mps"},"Desktop Companion app to request MPS"),", the MPS CLI provides more features and more robust file uploading. This page provides basic information to get you started with the MPS CLI, go to the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli_guide"},"MPS CLI Guide")," for more details."),(0,o.mdx)("p",null,"The MPS inputs can be a file or directory, and multiple inputs can be listed in a single command."),(0,o.mdx)("p",null,"The MPS CLI has two modes:"),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},"Single",(0,o.mdx)("ul",{parentName:"li"},(0,o.mdx)("li",{parentName:"ul"},"Process each recording individually"),(0,o.mdx)("li",{parentName:"ul"},"Output is always saved next to the input VRS file"),(0,o.mdx)("li",{parentName:"ul"},"The most common way to request MPS"))),(0,o.mdx)("li",{parentName:"ul"},"Multi",(0,o.mdx)("ul",{parentName:"li"},(0,o.mdx)("li",{parentName:"ul"},"Process multiple recordings to generate multi-sequence SLAM data"),(0,o.mdx)("li",{parentName:"ul"},"User must provide a directory for the outputs")))),(0,o.mdx)("admonition",{title:"Non-UI options available",type:"tip"},(0,o.mdx)("p",{parentName:"admonition"},"This tutorial uses the the MPS CLI UI, but all processes can also work can without using the UI and can be integrated into automated workflows. See the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/request_mps/mps_cli_guide#command-line-reference"},"Command Line Reference")," in the User Guide for more details.")),(0,o.mdx)("h2",{id:"getting-started"},"Getting Started"),(0,o.mdx)("h3",{id:"install-project-aria-tools"},"Install Project Aria Tools"),(0,o.mdx)("p",null,"Project Aria MPS CLI is only available if you install the ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/installation/installation_python"},"pip installation version of Project Aria Tools"),". This installation has been designed to be simple to use, even if you are not familiar with programming languages."),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_utilities/installation/installation_python"},"Install Project Aria Tools"))),(0,o.mdx)("p",null,"To return to your installation of Project Aria Tools at any time, restart the virtual environment using the following command:"),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre"},"source $HOME/projectaria_tools_python_env/bin/activate\n")),(0,o.mdx)("h3",{id:"download-the-mps-cli-sample-dataset"},"Download the MPS CLI sample dataset"),(0,o.mdx)("p",null,"To try out the following commands on VRS files:"),(0,o.mdx)("ol",null,(0,o.mdx)("li",{parentName:"ol"},"Download the sample files:",(0,o.mdx)("ul",{parentName:"li"},(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"https://www.projectaria.com/async/sample/download/?bucket=mps&filename=sample_multi_slam_1.vrs"},"Sample 1")," - single VRS file"),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"https://www.projectaria.com/async/sample/download/?bucket=mps&filename=sample_multi_slam_2.vrs"},"Sample 2")," - single VRS file"))),(0,o.mdx)("li",{parentName:"ol"},"Move them to a directory called Example in your downloads directory")),(0,o.mdx)("admonition",{type:"info"},(0,o.mdx)("p",{parentName:"admonition"},"You may also wish to use your own recordings.")),(0,o.mdx)("h3",{id:"request-mps-for-all-vrs-files-in-the-example-directory-and-its-subdirectories"},"Request MPS for all VRS files in the \u201cExample\u201d directory and it\u2019s subdirectories:"),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre"},"aria_mps single -i ~/Downloads/Example/\n")),(0,o.mdx)("p",null,"You'll be prompted to enter your username and password. Use the Project Aria credentials you use to sign into the Mobile Companion app."),(0,o.mdx)("p",null,(0,o.mdx)("img",{alt:"MPS CLU UI for single mode",src:a(14424).A,width:"2172",height:"1034"})),(0,o.mdx)("p",null,"Once the request has been processed, the MPS output will be downloaded next to the original VRS file. In this example, a recording in the Example directory called recording1.vrs was used to generate MPS."),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre"},"    \u2514\u2500\u2500 Example folder\n        \u251c\u2500\u2500 mps_recording1_vrs\n        \u2502   \u251c\u2500\u2500 eye_gaze\n        \u2502   \u2502   \u251c\u2500\u2500 general_eye_gaze.csv\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 slam\n        \u2502   \u2502   \u251c\u2500\u2500 closed_loop_trajectory.csv\n        \u2502   \u2502   \u251c\u2500\u2500 online_calibration.jsonl\n        \u2502   \u2502   \u251c\u2500\u2500 open_loop_trajectory.csv\n        \u2502   \u2502   \u251c\u2500\u2500 semidense_observations.csv.gz\n        \u2502   \u2502   \u251c\u2500\u2500 semidense_points.csv.gz\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 hand_tracking\n        \u2502   \u2502   \u251c\u2500\u2500 wrist_and_palm_poses.csv\n        \u2502   \u2502   \u2514\u2500\u2500 summary.json\n        \u2502   \u251c\u2500\u2500 vrs_health_check.json\n        \u2502   \u2514\u2500\u2500 vrs_health_check_slam.json\n        \u2514\u2500\u2500 recording1.vrs\n")),(0,o.mdx)("p",null,"Go to ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/mps_summary"},"MPS Data Format Basics")," for more details about the folder structure."),(0,o.mdx)("h3",{id:"exit-the-mps-cli"},"Exit the MPS CLI"),(0,o.mdx)("p",null,"To quit the MPS CLI, press ",(0,o.mdx)("inlineCode",{parentName:"p"},"CTRL + Q"),". The CLI will ask for confirmation before quitting."),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},"If you quit the request tool while the files are uploading the uploads will stop."),(0,o.mdx)("li",{parentName:"ul"},"If you resubmit the request the uploads will resume where they left off, progress won\u2019t be lost.")),(0,o.mdx)("p",null,"If you quit the request tool once the files have been uploaded, the MPS processes will continue. Once processing is complete, and the request tool is open, MPS files will be automatically downloaded to your VRS files\u2019 location."),(0,o.mdx)("h3",{id:"multi-sequence-mps-requests"},"Multi Sequence MPS Requests"),(0,o.mdx)("p",null,"When you request MPS using ",(0,o.mdx)("inlineCode",{parentName:"p"},"multi")," mode, MPS will process a group of recordings together to generate Multi-SLAM MPS. ",(0,o.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/mps/slam/mps_multi_slam"},"Multi-SLAM MPS")," creates SLAM MPS outputs in a shared co-ordinate frame. Once the request has been processed, the MPS output will be downloaded to the directory you defined."),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre"},"aria_mps multi -i ~/Downloads/Example/ -o ~/Documents/multi_slam_output\n")),(0,o.mdx)("p",null,(0,o.mdx)("img",{alt:"MPS CLU UI for single mode",src:a(78229).A,width:"2186",height:"1098"})),(0,o.mdx)("h2",{id:"working-with-mps-data"},"Working with MPS data"),(0,o.mdx)("p",null,"You may find the following resources helpful when working with MPS data:"),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_formats/mps/mps_summary"},"MPS Data Formats")),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_python"},"Visualize MPS Using Python")),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_cpp"},"Visualize MPS Using C++"))))}u.isMDXComponent=!0},78229:(e,t,a)=>{a.d(t,{A:()=>r});const r=a.p+"assets/images/mps_cli_ui_multi-a43800444b57753f41baa60097a68b1b.png"},14424:(e,t,a)=>{a.d(t,{A:()=>r});const r=a.p+"assets/images/mps_cli_ui_single-dc99c169a7136f49d7659b8df65e2e39.png"}}]);