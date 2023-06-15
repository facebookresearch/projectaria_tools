"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[6176],{3905:(e,t,a)=>{a.r(t),a.d(t,{MDXContext:()=>p,MDXProvider:()=>c,mdx:()=>f,useMDXComponents:()=>d,withMDXComponents:()=>u});var r=a(67294);function o(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function n(){return n=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var a=arguments[t];for(var r in a)Object.prototype.hasOwnProperty.call(a,r)&&(e[r]=a[r])}return e},n.apply(this,arguments)}function i(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,r)}return a}function s(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?i(Object(a),!0).forEach((function(t){o(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):i(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function l(e,t){if(null==e)return{};var a,r,o=function(e,t){if(null==e)return{};var a,r,o={},n=Object.keys(e);for(r=0;r<n.length;r++)a=n[r],t.indexOf(a)>=0||(o[a]=e[a]);return o}(e,t);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);for(r=0;r<n.length;r++)a=n[r],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(o[a]=e[a])}return o}var p=r.createContext({}),u=function(e){return function(t){var a=d(t.components);return r.createElement(e,n({},t,{components:a}))}},d=function(e){var t=r.useContext(p),a=t;return e&&(a="function"==typeof e?e(t):s(s({},t),e)),a},c=function(e){var t=d(e.components);return r.createElement(p.Provider,{value:t},e.children)},m="mdxType",h={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},v=r.forwardRef((function(e,t){var a=e.components,o=e.mdxType,n=e.originalType,i=e.parentName,p=l(e,["components","mdxType","originalType","parentName"]),u=d(a),c=o,m=u["".concat(i,".").concat(c)]||u[c]||h[c]||n;return a?r.createElement(m,s(s({ref:t},p),{},{components:a})):r.createElement(m,s({ref:t},p))}));function f(e,t){var a=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var n=a.length,i=new Array(n);i[0]=v;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s[m]="string"==typeof e?e:o,i[1]=s;for(var p=2;p<n;p++)i[p]=a[p];return r.createElement.apply(null,i)}return r.createElement.apply(null,a)}v.displayName="MDXCreateElement"},79555:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>l,contentTitle:()=>i,default:()=>c,frontMatter:()=>n,metadata:()=>s,toc:()=>p});var r=a(87462),o=(a(67294),a(3905));const n={sidebar_position:20,title:"Getting Started"},i=void 0,s={unversionedId:"data_utilities/getting_started",id:"data_utilities/getting_started",title:"Getting Started",description:"In this guide, we introduce how to install the projectaria_tools python package and provide tutorials to walk through the APIs to easily access and visualize Aria data.",source:"@site/docs/data_utilities/getting_started.mdx",sourceDirName:"data_utilities",slug:"/data_utilities/getting_started",permalink:"/projectaria_tools/docs/data_utilities/getting_started",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_utilities/getting_started.mdx",tags:[],version:"current",sidebarPosition:20,frontMatter:{sidebar_position:20,title:"Getting Started"},sidebar:"tutorialSidebar",previous:{title:"Overview",permalink:"/projectaria_tools/docs/data_utilities/"},next:{title:"Visualizers",permalink:"/projectaria_tools/docs/data_utilities/visualization_guide"}},l={},p=[{value:"Step 0 : Check system requirements and download codebase",id:"step-0--check-system-requirements-and-download-codebase",level:2},{value:"Step 1 : Install Python",id:"step-1--install-python",level:2},{value:"Step 2 : Create a virtual environment",id:"step-2--create-a-virtual-environment",level:2},{value:"Step 3 : Install projectaria_tools from pypi",id:"step-3--install-projectaria_tools-from-pypi",level:2},{value:"Step 4: Run Dataprovider quickstart tutorial",id:"step-4-run-dataprovider-quickstart-tutorial",level:2},{value:"Step 5: Run Machine Perception Services (MPS) quickstart tutorial",id:"step-5-run-machine-perception-services-mps-quickstart-tutorial",level:2},{value:"Troubleshooting",id:"troubleshooting",level:2},{value:"Other Useful Links",id:"other-useful-links",level:2}],u={toc:p},d="wrapper";function c(e){let{components:t,...a}=e;return(0,o.mdx)(d,(0,r.Z)({},u,a,{components:t,mdxType:"MDXLayout"}),(0,o.mdx)("p",null,"In this guide, we introduce how to install the projectaria_tools python package and provide tutorials to walk through the APIs to easily access and visualize Aria data.\nYou will run through two jupyter notebook tutorials:"),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("strong",{parentName:"li"},"Dataprovider quickstart tutorial"),": a walk-through of accessing sensor data from ",(0,o.mdx)("a",{parentName:"li",href:"/docs/data_formats/aria_vrs/aria_vrs_format"},"VRS file"),",\nobtaining sensor calibrations and accessing project/unproject functionalities, undistorting an image, etc."),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("strong",{parentName:"li"},"Machine Perception Services (MPS) quickstart tutorial"),": a guide to visualize MPS data\n(",(0,o.mdx)("a",{parentName:"li",href:"/docs/data_formats/mps/mps_eye_gaze"},"gaze"),", ",(0,o.mdx)("a",{parentName:"li",href:"/docs/data_formats/mps/mps_trajectory"},"trajectory"),", and ",(0,o.mdx)("a",{parentName:"li",href:"/docs/data_formats/mps/mps_pointcloud"},"point cloud"),"), etc.")),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/docs/data_utilities/installation/troubleshooting#jupyter-notebook-error"},(0,o.mdx)("strong",{parentName:"a"},"Jupyter notebook error"))," : If you have problems using Jupyter examples, please ",(0,o.mdx)("a",{parentName:"p",href:"https://www.python.org/downloads/"},"upgrade python3")," to the latest version."),(0,o.mdx)("hr",null),(0,o.mdx)("h2",{id:"step-0--check-system-requirements-and-download-codebase"},"Step 0 : Check system requirements and download codebase"),(0,o.mdx)("p",null,(0,o.mdx)("a",{parentName:"p",href:"/docs/data_utilities/installation/download_codebase"},"Ensure your system is supported and then download projectaria_tools codebase from the github")),(0,o.mdx)("h2",{id:"step-1--install-python"},"Step 1 : Install Python"),(0,o.mdx)("p",null,"Ensure python3 is installed on the system (check with ",(0,o.mdx)("inlineCode",{parentName:"p"},"python3 --version)")),(0,o.mdx)("h2",{id:"step-2--create-a-virtual-environment"},"Step 2 : Create a virtual environment"),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre",className:"language-bash"},"rm -rf $HOME/projectaria_tools_python_env\n\npython3 -m venv $HOME/projectaria_tools_python_env\n\nsource $HOME/projectaria_tools_python_env/bin/activate\n")),(0,o.mdx)("h2",{id:"step-3--install-projectaria_tools-from-pypi"},"Step 3 : Install projectaria_tools from pypi"),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre",className:"language-bash"},"pip3 install --upgrade pip\n\npip3 install projectaria-tools'[all]'\n")),(0,o.mdx)("h2",{id:"step-4-run-dataprovider-quickstart-tutorial"},"Step 4: Run Dataprovider quickstart tutorial"),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre",className:"language-bash"},"cd $HOME/Documents/projectaria_sandbox\n\njupyter notebook projectaria_tools/core/examples/dataprovider_quickstart_tutorial.ipynb\n")),(0,o.mdx)("h2",{id:"step-5-run-machine-perception-services-mps-quickstart-tutorial"},"Step 5: Run Machine Perception Services (MPS) quickstart tutorial"),(0,o.mdx)("p",null,"In the MPS tutorial, the notebook walks through how to visualize gaze, trajectory, and point cloud from MPS data."),(0,o.mdx)("pre",null,(0,o.mdx)("code",{parentName:"pre",className:"language-bash"},"cd $HOME/Documents/projectaria_sandbox\n\njupyter notebook projectaria_tools/core/examples/mps_quickstart_tutorial.ipynb\n")),(0,o.mdx)("h2",{id:"troubleshooting"},"Troubleshooting"),(0,o.mdx)("p",null,"Check the ",(0,o.mdx)("a",{parentName:"p",href:"/docs/data_utilities/installation/troubleshooting"},"troubleshooting")," if you are having issues in this guide."),(0,o.mdx)("h2",{id:"other-useful-links"},"Other Useful Links"),(0,o.mdx)("ul",null,(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/docs/data_utilities/installation/troubleshooting"},"Troubleshooting")),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/docs/data_utilities/installation/download_codebase"},"Installation guide")),(0,o.mdx)("li",{parentName:"ul"},(0,o.mdx)("a",{parentName:"li",href:"/projectaria_tools/docs/data_utilities/visualization_guide"},"Visualizers"))))}c.isMDXComponent=!0}}]);