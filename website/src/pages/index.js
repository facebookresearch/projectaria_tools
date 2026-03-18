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

import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

export default function Home() {
  const context = useDocusaurusContext();
  const {siteConfig = {}} = context;

  return (
    <Layout title={siteConfig.title} description={siteConfig.tagline}>
      <header className={clsx(styles.heroBanner)}>
        <div
          style={{
            justifyContent: 'center',
            display: 'flex',
          }}>
          <div className={clsx(styles.mainLogo)} />
        </div>
      </header>

      <main id="main" role="main">
        <div
          style={{
            textAlign: 'center',
            margin: '0 auto 1.5rem',
            padding: '0.75rem 1.5rem',
            maxWidth: '700px',
            backgroundColor: 'var(--ifm-color-emphasis-100)',
            borderRadius: '8px',
            border: '1px solid var(--ifm-color-emphasis-300)',
          }}>
          <p style={{margin: 0, fontSize: '1.1rem'}}>
            📌 This is the <strong>Aria Gen 1</strong> documentation site.
          </p>
          <p style={{margin: '0.5rem 0 0'}}>
            Looking for Aria Gen 2? Visit the{' '}
            <a href="https://facebookresearch.github.io/projectaria_tools/gen2/">
              Aria Gen 2 Documentation
            </a>
            .
          </p>
        </div>
        <p className={clsx(styles.tagLine, 'hero__subtitle')}>
          {siteConfig.tagline}
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx(
              'button button--outline button--secondary button--lg',
              styles.getStarted,
            )}
            to={useBaseUrl('docs/intro')}>
            GET STARTED
          </Link>
        </div>
      </main>
    </Layout>
  );
}
