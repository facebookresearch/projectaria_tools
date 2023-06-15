"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[8076],{3905:(t,e,n)=>{n.r(e),n.d(e,{MDXContext:()=>c,MDXProvider:()=>p,mdx:()=>h,useMDXComponents:()=>u,withMDXComponents:()=>d});var a=n(67294);function r(t,e,n){return e in t?Object.defineProperty(t,e,{value:n,enumerable:!0,configurable:!0,writable:!0}):t[e]=n,t}function i(){return i=Object.assign||function(t){for(var e=1;e<arguments.length;e++){var n=arguments[e];for(var a in n)Object.prototype.hasOwnProperty.call(n,a)&&(t[a]=n[a])}return t},i.apply(this,arguments)}function o(t,e){var n=Object.keys(t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(t);e&&(a=a.filter((function(e){return Object.getOwnPropertyDescriptor(t,e).enumerable}))),n.push.apply(n,a)}return n}function s(t){for(var e=1;e<arguments.length;e++){var n=null!=arguments[e]?arguments[e]:{};e%2?o(Object(n),!0).forEach((function(e){r(t,e,n[e])})):Object.getOwnPropertyDescriptors?Object.defineProperties(t,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(e){Object.defineProperty(t,e,Object.getOwnPropertyDescriptor(n,e))}))}return t}function l(t,e){if(null==t)return{};var n,a,r=function(t,e){if(null==t)return{};var n,a,r={},i=Object.keys(t);for(a=0;a<i.length;a++)n=i[a],e.indexOf(n)>=0||(r[n]=t[n]);return r}(t,e);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(t);for(a=0;a<i.length;a++)n=i[a],e.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(t,n)&&(r[n]=t[n])}return r}var c=a.createContext({}),d=function(t){return function(e){var n=u(e.components);return a.createElement(t,i({},e,{components:n}))}},u=function(t){var e=a.useContext(c),n=e;return t&&(n="function"==typeof t?t(e):s(s({},e),t)),n},p=function(t){var e=u(t.components);return a.createElement(c.Provider,{value:e},t.children)},m="mdxType",b={inlineCode:"code",wrapper:function(t){var e=t.children;return a.createElement(a.Fragment,{},e)}},f=a.forwardRef((function(t,e){var n=t.components,r=t.mdxType,i=t.originalType,o=t.parentName,c=l(t,["components","mdxType","originalType","parentName"]),d=u(n),p=r,m=d["".concat(o,".").concat(p)]||d[p]||b[p]||i;return n?a.createElement(m,s(s({ref:e},c),{},{components:n})):a.createElement(m,s({ref:e},c))}));function h(t,e){var n=arguments,r=e&&e.mdxType;if("string"==typeof t||r){var i=n.length,o=new Array(i);o[0]=f;var s={};for(var l in e)hasOwnProperty.call(e,l)&&(s[l]=e[l]);s.originalType=t,s[m]="string"==typeof t?t:r,o[1]=s;for(var c=2;c<i;c++)o[c]=n[c];return a.createElement.apply(null,o)}return a.createElement.apply(null,n)}f.displayName="MDXCreateElement"},61805:(t,e,n)=>{n.r(e),n.d(e,{assets:()=>l,contentTitle:()=>o,default:()=>p,frontMatter:()=>i,metadata:()=>s,toc:()=>c});var a=n(87462),r=(n(67294),n(3905));const i={sidebar_position:10,title:"Attribution and Contributing"},o=void 0,s={unversionedId:"attribution_citation/attribution_citation",id:"attribution_citation/attribution_citation",title:"Attribution and Contributing",description:"Citation",source:"@site/docs/attribution_citation/attribution_citation.mdx",sourceDirName:"attribution_citation",slug:"/attribution_citation/",permalink:"/projectaria_tools/docs/attribution_citation/",draft:!1,editUrl:"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/attribution_citation/attribution_citation.mdx",tags:[],version:"current",sidebarPosition:10,frontMatter:{sidebar_position:10,title:"Attribution and Contributing"},sidebar:"tutorialSidebar",previous:{title:"Temporal Alignment of Sensor Data",permalink:"/projectaria_tools/docs/tech_insights/temporal_alignment_of_sensor_data"}},l={},c=[{value:"Citation",id:"citation",level:2},{value:"Project Aria Tools",id:"project-aria-tools",level:3},{value:"Aria Digital Twin Dataset",id:"aria-digital-twin-dataset",level:3},{value:"Aria Synthetic Environments Dataset",id:"aria-synthetic-environments-dataset",level:3},{value:"Aria Pilot Dataset",id:"aria-pilot-dataset",level:3}],d={toc:c},u="wrapper";function p(t){let{components:e,...n}=t;return(0,r.mdx)(u,(0,a.Z)({},d,n,{components:e,mdxType:"MDXLayout"}),(0,r.mdx)("h2",{id:"citation"},"Citation"),(0,r.mdx)("h3",{id:"project-aria-tools"},"Project Aria Tools"),(0,r.mdx)("p",null,"If you use Project Aria tools or data in your research, please consider starring \u2b50 our ",(0,r.mdx)("a",{parentName:"p",href:"https://github.com/facebookresearch/projectaria_tools"},"github repository")," and citing the Project Aria Whitepaper (details are coming soon)."),(0,r.mdx)("h3",{id:"aria-digital-twin-dataset"},"Aria Digital Twin Dataset"),(0,r.mdx)("p",null,"If you use Aria Digital Twin dataset and its tools, please cite the ",(0,r.mdx)("a",{parentName:"p",href:"https://arxiv.org/abs/2306.06362"},"Aria Digital Twin dataset paper"),":"),(0,r.mdx)("pre",null,(0,r.mdx)("code",{parentName:"pre"},"@misc{pan2023aria,\n      title={Aria Digital Twin: A New Benchmark Dataset for Egocentric 3D Machine Perception},\n      author={Xiaqing Pan and Nicholas Charron and Yongqian Yang and Scott Peters and Thomas Whelan and Chen Kong and Omkar Parkhi and Richard Newcombe and Carl Yuheng Ren},\n      year={2023},\n      eprint={2306.06362},\n      archivePrefix={arXiv},\n      primaryClass={cs.CV}\n}\n")),(0,r.mdx)("h3",{id:"aria-synthetic-environments-dataset"},"Aria Synthetic Environments Dataset"),(0,r.mdx)("p",null,"If you use Aria Synthetic Environments Dataset and its tools, please cite the Aria Synthetic Environments dataset paper (coming soon)."),(0,r.mdx)("h3",{id:"aria-pilot-dataset"},"Aria Pilot Dataset"),(0,r.mdx)("p",null,"If you use the Aria Pilot Dataset in GitHub, please cite"),(0,r.mdx)("pre",null,(0,r.mdx)("code",{parentName:"pre"},"@misc{aria_pilot_dataset,\n    title           = {Aria Pilot Dataset},\n    author          = {Zhaoyang Lv and Edward Miller and Jeff Meissner and Luis Pesqueira and\n    Chris Sweeney and Jing Dong and Lingni Ma and Pratik Patel and Pierre Moulon and\n    Kiran Somasundaram and Omkar Parkhi and Yuyang Zou and Nikhil Raina and Steve Saarinen\n    and Yusuf M Mansour and Po-Kang Huang and Zijian Wang and Anton Troynikov and Raul Mur Artal\n    and Daniel DeTone and Daniel Barnes and Elizabeth Argall and Andrey Lobanovskiy and\n    David Jaeyun Kim and Philippe Bouttefroy and Julian Straub and Jakob Julian Engel and\n    Prince Gupta and Mingfei Yan and Renzo De Nardi and Richard Newcombe},\n    howpublished    = {\\url{https://about.facebook.com/realitylabs/projectaria/datasets}},\n    year            = {2022}\n}\n")))}p.isMDXComponent=!0}}]);