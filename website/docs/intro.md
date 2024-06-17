---
sidebar_position: 1
id: intro
title: Introduction to Project Aria Docs
---


**Project Aria** is a research platform developed by Meta Reality Labs Research to enable both internal groups in Meta and external researchers to use egocentric data to push the state of the art in egocentric AI research, including the subfields of computer vision, robotics, and contextual AI, as examples. We believe it will take many of the world’s leading research communities to collectively solve the biggest problems in this deep research domain.

Project Aria’s offering to the external community is organized into two major pillars:

* **Open Science Initiatives** are a collection of open-sourced egocentric datasets, models for egocentric AI applications, and tooling for working with the datasets that you can download and start experimenting with right now. We also have a set of challenges that we pose to the research community for solving AI problems.
* The **Aria Research Kit** is a specialized toolkit for egocentric data collection. The kit comes with the Aria glasses for multimodal egocentric data capture, a mobile companion app for operating the glasses, applications for dataset management and review, a command-line interface for programmatically operating the glasses, Machine Perception Services (MPS) for post-processing collected data, and a Client SDK for the development of custom applications and integrations. Qualified academic and commercial research partners can apply to obtain the [Aria Research Kit](https://www.projectaria.com/research-kit/).

![About Project Aria, showing services provided, simliar to what is in the FAQ](/img/intro.png)

## New to Project Aria?

* Go to [projectaria.com](http://projectaria.com) to get an overview of the program
* Go to [Aria Dataset Explorer](https://explorer.projectaria.com/) to search and preview some of Aria’s open data
* Go to the [Project Aria FAQ](faq.mdx) for an overview of our current service offerings, capabilities and software ecosystem

## Want to get involved with Open Science Initiatives (OSI)?

We believe that open source accelerates the pace of innovation in the world. We’re excited to share our code, models and data and collaborate together on Open Science that can help shape the future.

Open datasets powered by Project Aria data include:

* [Aria Everyday Activities (AEA)](/open_datasets/aria_everyday_activities_dataset/aria_everyday_activities_dataset.mdx) - a re-release of Aria’s first Pilot Dataset, updated with new tooling and location data, to accelerate the state of machine perception and AI.
* [Aria Digital Twin (ADT)](https://www.projectaria.com/datasets/adt/) - a real-world dataset, with hyper-accurate digital counterpart & comprehensive ground-truth annotation
* [Aria Synthetic Environments (ASE)](https://www.projectaria.com/datasets/ase/) - a procedurally-generated synthetic Aria dataset for large-scale ML research
* [HOT3D](https://www.projectaria.com/datasets/hot3d/) - a new benchmark dataset for vision-based understanding of 3D hand-object interactions
* [Nymeria](https://www.projectaria.com/datasets/nymeria/) - a large-scale multimodal egocentric dataset for full-body motion understanding


Research challenges, using our open datasets, are posted to [projectaria.com](https://www.projectaria.com/challenges/).

Models created using Aria data include:
* [EgoBlur](https://www.projectaria.com/tools/egoblur/) - an open source AI model from Meta to preserve privacy by detecting and blurring PII from images. Designed to work with egocentric data (such as Aria data) and non-egocentric data.
* [Project Aria Eye Tracking](https://github.com/facebookresearch/projectaria_eyetracking) - an open source inference code for the [Pre March 2024 Eye Gaze Model](/data_formats/mps/mps_eye_gaze.mdx) used by Machine Perception Services (MPS)

Aria data is recorded using [VRS](/data_formats/aria_vrs/aria_vrs.mdx), an open source file format. Our open source code, [Project Aria Tools](/data_utilities/data_utilities.mdx), provides a C++ and Python interface that helps people incorporate VRS data into a wide range of downstream applications.


## Interested in getting access to the Aria Research Kit (ARK)?

Through our OSI data, tooling and research challenges we aim to support the broadest audience of the research community. For researchers who also need access to a [physical Project Aria device](/tech_spec/hardware_spec.mdx), we offer the [Aria Research Kit (ARK)](https://www.projectaria.com/research-kit/).

Project Aria devices can be used to:



* Collect data
    * In addition to capturing data from a single device, TICSync can be used to capture time synchronized data from multiple Aria devices in a shared world location
    * Cloud based Machine Perception Services (MPS) are available to generate SLAM, Multi-SLAM, Eye Gaze and Hand Tracking derived data outputs. Partner data is only used to serve MPS requests. Partner data is not available to Meta researchers or Meta’s affiliates.
* Stream data
    * Use the Project Aria Client SDK to stream and subscribe to data on your local machine

Before applying for the ARK, please explore our OSI offerings, [FAQ](/faq.mdx). Once you are confident that the ARK is a good match for your research, please apply for the Aria Research Kit.

* [Project Aria: Academic Partnership Interest Form](https://www.facebook.com/help/contact/409561724891076)
* [Project Aria: Corporate Partnership Interest Form](https://docs.google.com/forms/d/e/1FAIpQLSeEQkP6zM-T2mrn5WUy2K-CliiXPXXmgHUEmT20FtAk5fi6vw/viewform)

Our team will review your application and reach out to you with next steps if you are approved for the ARK.


## Just received Project Aria glasses?

* [About ARK](/ARK/about_ARK.mdx) provides an overview of different ways you can use the device
* The [Quickstart Guide](/ARK/ARK_quickstart.mdx) covers how to set up your glasses
* The [Glasses User Manual](/ARK/glasses_manual/glasses_user_manual.mdx) provides a range of information, including how to factory reset your glasses

If you encounter any issues, go to our [Support page](/support.mdx) for multiple ways to get in touch!
