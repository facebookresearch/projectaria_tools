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
 */

import React from 'react';

interface TutorialButtonsProps {
  notebookUrl: string;
  colabUrl?: string;
  colabDisabled?: boolean;
}

const TutorialButtons: React.FC<TutorialButtonsProps> = ({
  notebookUrl,
  colabUrl,
  colabDisabled = false,
}) => {
  const handleColabClick = () => {
    alert(
      'Google Colab integration is not available yet. Please download the notebook and run it locally.',
    );
  };

  const ColabIcon = () => (
    <svg
      style={{marginRight: '8px', width: '16px', height: '16px'}}
      fill="currentColor"
      viewBox="0 0 24 24">
      <path d="M16.9414 4.9757a7.033 7.033 0 0 0-4.9308 2.0324 7.033 7.033 0 0 0-.1232 9.8068l2.395-2.395a3.6455 3.6455 0 0 1 5.1497-5.1478l2.397-2.3989a7.033 7.033 0 0 0-4.8877-1.9175zM7.07 4.9855a7.033 7.033 0 0 0-4.8878 1.9175l2.395 2.3989a3.6434 3.6434 0 0 1 5.1497 5.1478l2.395 2.395a7.033 7.033 0 0 0-.1232-9.8068A7.033 7.033 0 0 0 7.07 4.9855zm15.0191 2.5923l-2.395 2.395a3.6455 3.6455 0 0 1-5.1497 5.1497l-2.395 2.395a7.033 7.033 0 0 0 9.9397-.123z" />
    </svg>
  );

  return (
    <div
      style={{
        display: 'flex',
        gap: '10px',
        marginBottom: '20px',
        flexWrap: 'wrap',
      }}>
      <a
        href={notebookUrl}
        target="_blank"
        rel="noopener noreferrer"
        style={{
          display: 'inline-flex',
          alignItems: 'center',
          padding: '8px 16px',
          backgroundColor: '#f6f8fa',
          border: '1px solid #d0d7de',
          borderRadius: '6px',
          textDecoration: 'none',
          color: '#24292f',
          fontSize: '14px',
          fontWeight: '500',
          transition: 'background-color 0.2s',
        }}
        onMouseEnter={(e) => {
          (e.target as HTMLElement).style.backgroundColor = '#f3f4f6';
        }}
        onMouseLeave={(e) => {
          (e.target as HTMLElement).style.backgroundColor = '#f6f8fa';
        }}>
        <svg
          style={{marginRight: '8px', width: '16px', height: '16px'}}
          fill="currentColor"
          viewBox="0 0 16 16">
          <path d="M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z" />
        </svg>
        View Notebook on GitHub
      </a>

      {!colabDisabled && colabUrl ? (
        <a
          href={colabUrl}
          target="_blank"
          rel="noopener noreferrer"
          style={{
            display: 'inline-flex',
            alignItems: 'center',
            padding: '8px 16px',
            backgroundColor: '#f9ab00',
            border: '1px solid #d0d7de',
            borderRadius: '6px',
            textDecoration: 'none',
            color: '#fff',
            fontSize: '14px',
            fontWeight: '500',
            transition: 'background-color 0.2s',
          }}
          onMouseEnter={(e) => {
            (e.target as HTMLElement).style.backgroundColor = '#e8a500';
          }}
          onMouseLeave={(e) => {
            (e.target as HTMLElement).style.backgroundColor = '#f9ab00';
          }}>
          <ColabIcon />
          Run in Google Colab
        </a>
      ) : (
        <button
          onClick={handleColabClick}
          style={{
            display: 'inline-flex',
            alignItems: 'center',
            padding: '8px 16px',
            backgroundColor: '#f6f8fa',
            border: '1px solid #d0d7de',
            borderRadius: '6px',
            color: '#656d76',
            fontSize: '14px',
            fontWeight: '500',
            cursor: 'not-allowed',
            transition: 'background-color 0.2s',
          }}>
          <ColabIcon />
          Colab (Coming Soon)
        </button>
      )}
    </div>
  );
};

export default TutorialButtons;
