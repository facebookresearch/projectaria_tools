/**
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @format
 */

// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

import math from 'remark-math';
import katex from 'rehype-katex';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Aria Gen 1 Docs',
  tagline:
    'Documentation and tooling to support researchers expand the horizons of Augmented Reality, Machine Perception and Artificial Intelligence',
  favicon: 'img/aria_logo.png',

  url: 'https://facebookresearch.github.io',
  baseUrl: '/projectaria_tools/',
  trailingSlash: false,

  organizationName: 'Facebook Research',
  projectName: 'projectaria_tools',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  presets: [
    [
      'docusaurus-plugin-internaldocs-fb/docusaurus-preset',
      /** @type {import('docusaurus-plugin-internaldocs-fb').PresetOptions} */
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/facebookresearch/projectaria_tools/tree/main/website/',
          remarkPlugins: [math],
          rehypePlugins: [katex],
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        gtag: {
          trackingID: 'G-Q1Q26XCR5N',
        },
      },
    ],
  ],
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.15.2/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-MlJdn/WNKDGXveldHDdyRP1R4CTHr3FeuDNfhsLPYrq2t0UBkUdK2jyTnXPEK1NQ',
      crossorigin: 'anonymous',
    },
  ],
  plugins: [
    [
      require.resolve('@cmfcmf/docusaurus-search-local'),
      {
        indexBlog: false,
        indexPages: true,
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    {
      announcementBar: {
        id: 'gen1_notice',
        content:
          '⚠️ This is the <strong>Aria Gen 1</strong> documentation site. For Aria Gen 2 docs, visit <a target="_blank" rel="noopener noreferrer" href="https://facebookresearch.github.io/projectaria_tools/gen2/">Aria Gen 2 Documentation</a>.',
        backgroundColor: '#FFF3CD',
        textColor: '#856404',
        isCloseable: false,
      },
      navbar: {
        title: 'Aria Gen 1 Docs',
        logo: {
          alt: 'aria-research-kit-sdk Logo',
          src: 'img/aria_logo.png',
        },
        items: [
          {
            href: 'https://www.projectaria.com/',
            label: 'Project Aria',
            position: 'left',
          },
          // Please keep GitHub link to the right for consistency.
          {
            href: 'https://github.com/facebookresearch/projectaria_tools',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introducing Project Aria',
                href: 'https://www.projectaria.com/',
              },
              {
                label: 'Aria Gen 1 Documentation',
                to: '/docs/intro',
              },
              {
                label: 'Aria Gen 2 Documentation',
                href: 'https://facebookresearch.github.io/projectaria_tools/gen2/',
              },
              {
                label: 'Get Support',
                to: '/docs/support',
              },
            ],
          },
          {
            title: 'Participate',
            items: [
              {
                label: 'Open Datasets',
                href: 'https://www.projectaria.com/datasets/',
              },
              {
                label: 'Challenges',
                href: 'https://www.projectaria.com/challenges/',
              },
              {
                label: 'Partner Program',
                href: 'https://www.projectaria.com/research-kit/',
              },
            ],
          },
          {
            title: 'Responsible Innovation',
            items: [
              {
                label: 'Innovation Principles',
                href: 'https://about.facebook.com/realitylabs/responsible-innovation-principles/',
              },
              {
                label: 'Project Aria Research Community Guidelines',
                href: 'https://about.facebook.com/realitylabs/projectaria/community-guidelines/',
              },
            ],
          },
          {
            title: 'Legal',
            // Please do not remove the privacy and terms, it's a legal requirement.
            items: [
              {
                label: 'Privacy',
                href: 'https://opensource.fb.com/legal/privacy/',
              },
              {
                label: 'Terms',
                href: 'https://opensource.fb.com/legal/terms/',
              },
              {
                label: 'Data Policy',
                href: 'https://opensource.fb.com/legal/data-policy/',
              },
              {
                label: 'Cookie Policy',
                href: 'https://opensource.fb.com/legal/cookie-policy/',
              },
            ],
          },
        ],
        logo: {
          alt: 'Meta Open Source Logo',
          // This default includes a positive & negative version, allowing for
          // appropriate use depending on your site's style.
          src: '/img/meta_opensource_logo_negative.svg',
          href: 'https://opensource.fb.com',
        },
        // Please do not remove the credits, help to publicize Docusaurus :)
        copyright: `Copyright © ${new Date().getFullYear()} Meta Platforms, Inc. Built with Docusaurus.`,
      },
    },
};

module.exports = config;
