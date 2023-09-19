"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[9100],{3905:(e,t,o)=>{o.r(t),o.d(t,{MDXContext:()=>c,MDXProvider:()=>u,mdx:()=>y,useMDXComponents:()=>p,withMDXComponents:()=>d});var n=o(67294);function r(e,t,o){return t in e?Object.defineProperty(e,t,{value:o,enumerable:!0,configurable:!0,writable:!0}):e[t]=o,e}function a(){return a=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var o=arguments[t];for(var n in o)Object.prototype.hasOwnProperty.call(o,n)&&(e[n]=o[n])}return e},a.apply(this,arguments)}function i(e,t){var o=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),o.push.apply(o,n)}return o}function l(e){for(var t=1;t<arguments.length;t++){var o=null!=arguments[t]?arguments[t]:{};t%2?i(Object(o),!0).forEach((function(t){r(e,t,o[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(o)):i(Object(o)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(o,t))}))}return e}function s(e,t){if(null==e)return{};var o,n,r=function(e,t){if(null==e)return{};var o,n,r={},a=Object.keys(e);for(n=0;n<a.length;n++)o=a[n],t.indexOf(o)>=0||(r[o]=e[o]);return r}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(n=0;n<a.length;n++)o=a[n],t.indexOf(o)>=0||Object.prototype.propertyIsEnumerable.call(e,o)&&(r[o]=e[o])}return r}var c=n.createContext({}),d=function(e){return function(t){var o=p(t.components);return n.createElement(e,a({},t,{components:o}))}},p=function(e){var t=n.useContext(c),o=t;return e&&(o="function"==typeof e?e(t):l(l({},t),e)),o},u=function(e){var t=p(e.components);return n.createElement(c.Provider,{value:t},e.children)},m="mdxType",f={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},b=n.forwardRef((function(e,t){var o=e.components,r=e.mdxType,a=e.originalType,i=e.parentName,c=s(e,["components","mdxType","originalType","parentName"]),d=p(o),u=r,m=d["".concat(i,".").concat(u)]||d[u]||f[u]||a;return o?n.createElement(m,l(l({ref:t},c),{},{components:o})):n.createElement(m,l({ref:t},c))}));function y(e,t){var o=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var a=o.length,i=new Array(a);i[0]=b;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l[m]="string"==typeof e?e:r,i[1]=l;for(var c=2;c<a;c++)i[c]=o[c];return n.createElement.apply(null,i)}return n.createElement.apply(null,o)}b.displayName="MDXCreateElement"},5520:(e,t,o)=>{o.r(t),o.d(t,{assets:()=>s,contentTitle:()=>i,default:()=>u,frontMatter:()=>a,metadata:()=>l,toc:()=>c});var n=o(87462),r=(o(67294),o(3905));const a={sidebar_position:10,title:"Download Codebase"},i="How to Download Project Aria Tools",l={unversionedId:"data_utilities/installation/download_codebase",id:"data_utilities/installation/download_codebase",title:"Download Codebase",description:"Supported Platforms",source:"@site/docs/data_utilities/installation/download_codebase.mdx",sourceDirName:"data_utilities/installation",slug:"/data_utilities/installation/download_codebase",permalink:"/projectaria_tools/docs/data_utilities/installation/download_codebase",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/data_utilities/installation/download_codebase.mdx",tags:[],version:"current",sidebarPosition:10,frontMatter:{sidebar_position:10,title:"Download Codebase"},sidebar:"tutorialSidebar",previous:{title:"Visualizers",permalink:"/projectaria_tools/docs/data_utilities/visualization_guide"},next:{title:"Python Package Installation",permalink:"/projectaria_tools/docs/data_utilities/installation/installation_python"}},s={},c=[{value:"Supported Platforms",id:"supported-platforms",level:2},{value:"Download codebase",id:"download-codebase",level:2}],d={toc:c},p="wrapper";function u(e){let{components:t,...o}=e;return(0,r.mdx)(p,(0,n.Z)({},d,o,{components:t,mdxType:"MDXLayout"}),(0,r.mdx)("h1",{id:"how-to-download-project-aria-tools"},"How to Download Project Aria Tools"),(0,r.mdx)("h2",{id:"supported-platforms"},"Supported Platforms"),(0,r.mdx)("p",null,"The codebase has been tested on the following platforms."),(0,r.mdx)("ul",null,(0,r.mdx)("li",{parentName:"ul"},"Fedora 36,37,38 recommended"),(0,r.mdx)("li",{parentName:"ul"},"Mac Intel / Mac ARM-based (M1)"),(0,r.mdx)("li",{parentName:"ul"},"Ubuntu focal (20.04 LTS) and jammy (22.04 LTS)")),(0,r.mdx)("h2",{id:"download-codebase"},"Download codebase"),(0,r.mdx)("pre",null,(0,r.mdx)("code",{parentName:"pre",className:"language-bash"},"mkdir -p $HOME/Documents/projectaria_sandbox\n\ncd $HOME/Documents/projectaria_sandbox\n\ngit clone https://github.com/facebookresearch/projectaria_tools.git\n")))}u.isMDXComponent=!0}}]);