"use strict";(self.webpackChunkwebsite=self.webpackChunkwebsite||[]).push([[30],{49258:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>d,contentTitle:()=>r,default:()=>h,frontMatter:()=>o,metadata:()=>i,toc:()=>l});const i=JSON.parse('{"id":"open_datasets/aria_digital_twin_dataset/aria_digital_twin_dataset","title":"Overview","description":"Overview","source":"@site/docs/open_datasets/aria_digital_twin_dataset/aria_digital_twin_dataset.mdx","sourceDirName":"open_datasets/aria_digital_twin_dataset","slug":"/open_datasets/aria_digital_twin_dataset/","permalink":"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/","draft":false,"unlisted":false,"editUrl":"https://github.com/facebookresearch/projectaria_tools/tree/main/website/docs/open_datasets/aria_digital_twin_dataset/aria_digital_twin_dataset.mdx","tags":[],"version":"current","sidebarPosition":10,"frontMatter":{"sidebar_position":10,"title":"Overview"},"sidebar":"tutorialSidebar","previous":{"title":"Recording Scripts","permalink":"/projectaria_tools/docs/open_datasets/aria_everyday_activities_dataset/aea_scripts"},"next":{"title":"Getting Started","permalink":"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/getting_started"}}');var n=a(74848),s=a(28453);const o={sidebar_position:10,title:"Overview"},r="Aria Digital Twin Dataset",d={},l=[{value:"Overview",id:"overview",level:2},{value:"About the data",id:"about-the-data",level:2},{value:"Apartment scene",id:"apartment-scene",level:3},{value:"Office scene",id:"office-scene",level:3},{value:"Activities",id:"activities",level:3},{value:"Documentation",id:"documentation",level:3}];function c(e){const t={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",h3:"h3",header:"header",li:"li",p:"p",ul:"ul",...(0,s.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsx)(t.header,{children:(0,n.jsx)(t.h1,{id:"aria-digital-twin-dataset",children:"Aria Digital Twin Dataset"})}),"\n",(0,n.jsx)(t.h2,{id:"overview",children:"Overview"}),"\n",(0,n.jsxs)(t.p,{children:["Project Aria Tools provides Python and C++ APIs to access the ",(0,n.jsx)(t.a,{href:"https://www.projectaria.com/datasets/adt/",children:"Aria Digital Twin (ADT) dataset"}),"\n(",(0,n.jsx)(t.a,{href:"https://openaccess.thecvf.com/content/ICCV2023/html/Pan_Aria_Digital_Twin_A_New_Benchmark_Dataset_for_Egocentric_3D_ICCV_2023_paper.html",children:"paper"}),")."]}),"\n",(0,n.jsx)(t.h2,{id:"about-the-data",children:"About the data"}),"\n",(0,n.jsxs)(t.p,{children:["ADT provides raw and synthesized sensor data from Project Aria glasses, combined with groundtruth data generated using a motion capture\nsystem including depth images, device trajectories, object trajectories and bounding boxes, and human tracking.\nWe also provide processed sensor data from our ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/ARK/mps/",children:"Machine Perception Services"}),".\nGo to ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/data_format",children:"ADT Data Format"})," to see a full list of the data we provide."]}),"\n",(0,n.jsx)(t.p,{children:"The ADT dataset contains 236 sequences recording single and dual-person activities.\nThe data was recorded in two spaces: an apartment and a single room office.\nThere are 74 single-instance dynamic objects shared between the two spaces."}),"\n",(0,n.jsxs)(t.p,{children:["Go to the ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/getting_started",children:"Getting Started Tutorial"})," to explore the sample dataset (available on Google colab, no download necessary)\nor the ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/dataset_download",children:"Dataset Download page"})," to get started."]}),"\n",(0,n.jsx)(t.p,{children:"The sample dataset is a single-user dataset with body pose in the Apartment.\nIt is a pretty representative example that should give you an idea of the dataset."}),"\n",(0,n.jsxs)(t.p,{children:["In addition to the sensor data and annotated ground truth data, we also provide high quality 3D object models (in .glb) for each of the objects in ADT.\nFor more info on downloading and viewing these models, see ",(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/object_models",children:"Object Model"})," page."]}),"\n",(0,n.jsx)(t.admonition,{title:"NEW UPDATE!",type:"info",children:(0,n.jsx)(t.p,{children:"3D object model release is new as of September 2024"})}),"\n",(0,n.jsx)(t.h3,{id:"apartment-scene",children:"Apartment scene"}),"\n",(0,n.jsx)(t.p,{children:"284 sequences were recorded in the apartment scene. The apartment comprised of a living room, kitchen, dining room and bedroom and contained\n281 unique stationary objects."}),"\n",(0,n.jsx)(t.p,{children:"Given some objects have multiple instances that may differ slightly, the apartment has 324 stationary object instances in total."}),"\n",(0,n.jsx)(t.h3,{id:"office-scene",children:"Office scene"}),"\n",(0,n.jsx)(t.p,{children:"52 sequences were recorded in the office scene, a single room with minimal office furniture.\nThe office room contained 15 unique stationary objects and 20 stationary object instances."}),"\n",(0,n.jsx)(t.h3,{id:"activities",children:"Activities"}),"\n",(0,n.jsx)(t.p,{children:"In the office scene, users examined objects. For the apartment scene we designed five single-person activities and three dual-person activities."}),"\n",(0,n.jsx)(t.p,{children:"The single-person activities were:"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"Room decoration"}),"\n",(0,n.jsx)(t.li,{children:"Meal preparation"}),"\n",(0,n.jsx)(t.li,{children:"Work"}),"\n",(0,n.jsx)(t.li,{children:"Object examination"}),"\n",(0,n.jsx)(t.li,{children:"Room cleaning"}),"\n"]}),"\n",(0,n.jsx)(t.p,{children:"The dual-person activities included:"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"Partying"}),"\n",(0,n.jsx)(t.li,{children:"Room cleaning"}),"\n",(0,n.jsx)(t.li,{children:"Dining table cleaning"}),"\n"]}),"\n",(0,n.jsx)(t.p,{children:"Every activity has 10 to 50 sequences and the activity names are embedded into the sequence_names."}),"\n",(0,n.jsx)(t.admonition,{type:"info",children:(0,n.jsxs)(t.p,{children:["We provide a mix of datasets where users may or may not be wearing an Aria and/or a bodysuit.\nPlease refer to the ",(0,n.jsx)(t.code,{children:"skeleton_aria_association.json"})," to see the case for each specific sub-sequence."]})}),"\n",(0,n.jsx)(t.h3,{id:"documentation",children:"Documentation"}),"\n",(0,n.jsx)(t.p,{children:"The ADT section of the wiki covers:"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/getting_started",children:"Getting Started"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"A quickstart tutorial available as a Google colab project or install Project Aria Tools Python package to run locally, run the ADT notebook to access and visualize ground-truth data."}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/dataset_download",children:"Dataset Download"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsxs)(t.li,{children:["A walkthrough of using ",(0,n.jsx)(t.code,{children:"aria_dataset_downloader"})," to download the ADT dataset."]}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/object_models",children:"Object Models"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsxs)(t.li,{children:["Everything you need to know about the released ADT object models including: a walkthrough of using ",(0,n.jsx)(t.code,{children:"dtc_object_downloader"})," to download the ADT object models, and brief description of data content and how to visualize the models"]}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/data_format",children:"Data Format"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"How ADT data is organized and stored"}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/data_loader",children:"Data Loader"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"APIs to load ADT data with handy code snippets"}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/visualizers",children:"Visualizers"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"Compile and run our visualizer using an example that accesses ADT data in C++."}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/advanced_tutorials/",children:"Advanced tutorials"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/advanced_tutorials/multiperson_synchronization",children:"MultiPerson Synchronization"}),": A guide to learn how device synchronization works in ADT."]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/advanced_tutorials/depth_maps_to_pointcloud",children:"Depth Maps to Pointcloud"}),": An example notebook showing how to convert depth maps & RGB images from ADT to a combined colored pointcloud in the Scene frame."]}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(t.li,{children:[(0,n.jsx)(t.a,{href:"/projectaria_tools/docs/open_datasets/aria_digital_twin_dataset/adt_challenges",children:"ADT Challenges"}),"\n",(0,n.jsxs)(t.ul,{children:["\n",(0,n.jsx)(t.li,{children:"Learn more about the ADT Grand Challenge"}),"\n"]}),"\n"]}),"\n"]})]})}function h(e={}){const{wrapper:t}={...(0,s.R)(),...e.components};return t?(0,n.jsx)(t,{...e,children:(0,n.jsx)(c,{...e})}):c(e)}},28453:(e,t,a)=>{a.d(t,{R:()=>o,x:()=>r});var i=a(96540);const n={},s=i.createContext(n);function o(e){const t=i.useContext(s);return i.useMemo((function(){return"function"==typeof e?e(t):{...t,...e}}),[t,e])}function r(e){let t;return t=e.disableParentContext?"function"==typeof e.components?e.components(n):e.components||n:o(e.components),i.createElement(s.Provider,{value:t},e.children)}}}]);