"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[4681],{15680:(e,t,a)=>{a.r(t),a.d(t,{MDXContext:()=>c,MDXProvider:()=>d,mdx:()=>h,useMDXComponents:()=>m,withMDXComponents:()=>p});var r=a(96540);function n(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function o(){return o=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var a=arguments[t];for(var r in a)Object.prototype.hasOwnProperty.call(a,r)&&(e[r]=a[r])}return e},o.apply(this,arguments)}function i(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,r)}return a}function s(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?i(Object(a),!0).forEach((function(t){n(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):i(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function l(e,t){if(null==e)return{};var a,r,n=function(e,t){if(null==e)return{};var a,r,n={},o=Object.keys(e);for(r=0;r<o.length;r++)a=o[r],t.indexOf(a)>=0||(n[a]=e[a]);return n}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(r=0;r<o.length;r++)a=o[r],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(n[a]=e[a])}return n}var c=r.createContext({}),p=function(e){return function(t){var a=m(t.components);return r.createElement(e,o({},t,{components:a}))}},m=function(e){var t=r.useContext(c),a=t;return e&&(a="function"==typeof e?e(t):s(s({},t),e)),a},d=function(e){var t=m(e.components);return r.createElement(c.Provider,{value:t},e.children)},u="mdxType",f={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},b=r.forwardRef((function(e,t){var a=e.components,n=e.mdxType,o=e.originalType,i=e.parentName,c=l(e,["components","mdxType","originalType","parentName"]),p=m(a),d=n,u=p["".concat(i,".").concat(d)]||p[d]||f[d]||o;return a?r.createElement(u,s(s({ref:t},c),{},{components:a})):r.createElement(u,s({ref:t},c))}));function h(e,t){var a=arguments,n=t&&t.mdxType;if("string"==typeof e||n){var o=a.length,i=new Array(o);i[0]=b;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s[u]="string"==typeof e?e:n,i[1]=s;for(var c=2;c<o;c++)i[c]=a[c];return r.createElement.apply(null,i)}return r.createElement.apply(null,a)}b.displayName="MDXCreateElement"},59123:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>l,contentTitle:()=>i,default:()=>d,frontMatter:()=>o,metadata:()=>s,toc:()=>c});var r=a(58168),n=(a(96540),a(15680));const o={sidebar_position:50,title:"Calibration Data"},i="MPS output - Calibration",s={unversionedId:"data_formats/mps/slam/mps_calibration",id:"data_formats/mps/slam/mps_calibration",title:"Calibration Data",description:"Online calibration is generated as part of SLAM (Location in the Desktop Companion app) MPS requests.",source:"@site/docs/data_formats/mps/slam/mps_calibration.mdx",sourceDirName:"data_formats/mps/slam",slug:"/data_formats/mps/slam/mps_calibration",permalink:"/projectaria_tools/docs/data_formats/mps/slam/mps_calibration",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_formats/mps/slam/mps_calibration.mdx",tags:[],version:"current",sidebarPosition:50,frontMatter:{sidebar_position:50,title:"Calibration Data"},sidebar:"tutorialSidebar",previous:{title:"Semi-Dense Point Cloud",permalink:"/projectaria_tools/docs/data_formats/mps/slam/mps_pointcloud"},next:{title:"Multi-SLAM",permalink:"/projectaria_tools/docs/data_formats/mps/slam/mps_multi_slam"}},l={},c=[{value:"Online calibration",id:"online-calibration",level:2}],p={toc:c},m="wrapper";function d(e){let{components:t,...a}=e;return(0,n.mdx)(m,(0,r.A)({},p,a,{components:t,mdxType:"MDXLayout"}),(0,n.mdx)("h1",{id:"mps-output---calibration"},"MPS output - Calibration"),(0,n.mdx)("p",null,"Online calibration is generated as part of SLAM (Location in the Desktop Companion app) ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/ARK/mps/"},"MPS requests"),"."),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},(0,n.mdx)("inlineCode",{parentName:"li"},"online_calibration.jsonl")," file")),(0,n.mdx)("p",null,"The ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/visualization/visualization_python#run-aria-sensor-viewer"},"Aria Sensor Viewer")," displays the relative position and orientation of most Project Aria glasses sensors (cameras, IMUs, microphones, magnetometer & barometer) in a common reference."),(0,n.mdx)("p",null,"Static camera calibration may also be available in some datasets that include stationary cameras as well as moving Project Aria glasses."),(0,n.mdx)("ul",null,(0,n.mdx)("li",{parentName:"ul"},"The name of the static camera calibration .csv file may vary between projects")),(0,n.mdx)("h2",{id:"online-calibration"},"Online calibration"),(0,n.mdx)("p",null,"The JSONL file contains one json online calibration record per line. Each record is a json dict object that contains timestamp metadata and the result of online calibration for the cameras and IMUs. The calibration parameters contain ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/tech_insights/camera_intrinsic_models"},"intrinsics")," and ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_formats/coordinate_convention/3d_coordinate_frame_convention"},"extrinsics")," parameters for each sensor as well as a time offsets which best temporally align their data. For how to load and read online calibrations in Python and C++, please check the ",(0,n.mdx)("a",{parentName:"p",href:"/projectaria_tools/docs/data_utilities/core_code_snippets/mps#online-calibration"},"code example")))}d.isMDXComponent=!0}}]);