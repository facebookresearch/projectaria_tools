import React, {useEffect, useMemo, useState} from 'react';
import CodeBlock from '@theme/CodeBlock';
// If your CSS module is under src/css/, keep this import.
// If you moved it next to the component, change to "./pat_installer_chooser.module.css"
import styles from './pat_installer_chooser.module.css';

type OS = 'linux_mac' | 'windows';
type Pkg = 'pip' | 'source';
type Channel = 'stable' | 'dev';

const OS_OPTIONS: {key: OS; label: string}[] = [
  {key: 'linux_mac', label: 'Linux & macOS'},
  {key: 'windows', label: 'Windows'},
];

const PKG_OPTIONS: {key: Pkg; label: string}[] = [
  {key: 'pip', label: 'pip'},
  {key: 'source', label: 'build from source (C++ & Python)'},
];

const CHANNEL_OPTIONS: {key: Channel; label: string}[] = [
  {key: 'stable', label: 'Stable'},
  {key: 'dev', label: 'Dev'},
];

export default function PatInstallerChooser() {
  const [os, setOs] = useState<OS>('linux_mac');
  const [pkg, setPkg] = useState<Pkg>('pip');
  const [channel, setChannel] = useState<Channel>('stable');
  const [copied, setCopied] = useState(false);

  // pip only supports stable
  useEffect(() => {
    if (pkg === 'pip' && channel !== 'stable') setChannel('stable');
  }, [pkg, channel]);

  const codeLang = os === 'windows' ? 'powershell' : 'bash';

  // venv bootstrap snippets
  const venvUnix = useMemo(
    () =>
      String.raw`rm -rf $HOME/projectaria_tools_python_env
python3.12 -m venv $HOME/projectaria_tools_python_env
source $HOME/projectaria_tools_python_env/bin/activate`,
    [],
  );

  const venvWin = useMemo(
    () =>
      String.raw`Remove-Item -Recurse -Force $HOME\projectaria_tools_python_env
py -3 -m venv $HOME\projectaria_tools_python_env
& $HOME\projectaria_tools_python_env\Scripts\Activate.ps1`,
    [],
  );

  const command = useMemo(() => {
    const venv = os === 'windows' ? venvWin : venvUnix;

    if (pkg === 'pip') {
      if (os === 'windows') {
        return String.raw`${venv}

python3 -m pip install projectaria-tools'[all]'==1.7.0`;
      }
      return String.raw`${venv}

python3.12 -m pip install projectaria-tools'[all]'==2.0.0`;
    }

    // source
    if (os === 'windows') {
      if (channel === 'stable') {
        return String.raw`${venv}

# Clone the correct branch of projectaria_tools
git clone https://github.com/facebookresearch/projectaria_tools.git -b 1.7.0

# Then follow instructions in "Advanced Installation From Source Code" to build from source.`;
      }
      // dev
      return String.raw`${venv}

# Clone the correct branch of projectaria_tools
git clone https://github.com/facebookresearch/projectaria_tools.git -b gen1_legacy

# Then follow instructions in "Advanced Installation From Source Code" to build from source.`;
    } else {
      if (channel === 'stable') {
        return String.raw`${venv}

# Clone the correct branch of projectaria_tools
git clone https://github.com/facebookresearch/projectaria_tools.git -b 2.0.0

# Then follow instructions in "Advanced Installation From Source Code" to build from source.`;
      }
      // dev
      return String.raw`${venv}

# Clone the correct branch of projectaria_tools
git clone https://github.com/facebookresearch/projectaria_tools.git -b main

# Then follow instructions in "Advanced Installation From Source Code" to build from source.`;
    }
  }, [os, pkg, channel, venvUnix, venvWin]);

  const doCopy = async () => {
    try {
      await navigator.clipboard.writeText(command);
      setCopied(true);
      setTimeout(() => setCopied(false), 1200);
    } catch {
      const ta = document.createElement('textarea');
      ta.value = command;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      document.body.removeChild(ta);
      setCopied(true);
      setTimeout(() => setCopied(false), 1200);
    }
  };

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <div className={styles.title}>Select your preferences</div>
        <div className={styles.subtitle}>
          We’ll generate the right install command
        </div>
      </div>

      <div className={styles.grid}>
        {/* OS */}
        <div className={styles.group}>
          <div className={styles.label}>Operating System</div>
          <div
            className={styles.segmented}
            role="radiogroup"
            aria-label="Operating System">
            {OS_OPTIONS.map((o) => (
              <label
                key={o.key}
                className={`${styles.option} ${os === o.key ? styles.active : ''}`}>
                <input
                  type="radio"
                  name="os"
                  value={o.key}
                  checked={os === o.key}
                  onChange={() => setOs(o.key)}
                />
                {o.label}
              </label>
            ))}
          </div>
        </div>

        {/* Package */}
        <div className={styles.group}>
          <div className={styles.label}>Package</div>
          <div
            className={styles.segmented}
            role="radiogroup"
            aria-label="Package">
            {PKG_OPTIONS.map((p) => (
              <label
                key={p.key}
                className={`${styles.option} ${pkg === p.key ? styles.active : ''}`}>
                <input
                  type="radio"
                  name="pkg"
                  value={p.key}
                  checked={pkg === p.key}
                  onChange={() => setPkg(p.key)}
                />
                {p.label}
              </label>
            ))}
          </div>
        </div>

        {/* Channel */}
        <div className={styles.group}>
          <div className={styles.label}>Channel</div>
          <div
            className={styles.segmented}
            role="radiogroup"
            aria-label="Channel">
            {CHANNEL_OPTIONS.map((c) => {
              const disabled = pkg === 'pip' && c.key === 'dev';
              return (
                <label
                  key={c.key}
                  className={`${styles.option} ${channel === c.key ? styles.active : ''} ${
                    disabled ? styles.disabled : ''
                  }`}
                  title={
                    disabled
                      ? 'Dev channel is not supported for pip'
                      : undefined
                  }>
                  <input
                    type="radio"
                    name="channel"
                    value={c.key}
                    checked={channel === c.key}
                    onChange={() => !disabled && setChannel(c.key)}
                    disabled={disabled}
                  />
                  {c.label}
                </label>
              );
            })}
          </div>
          {pkg === 'pip' && (
            <div className={styles.helper}>
              Dev channel is unavailable when using pip.
            </div>
          )}
        </div>
      </div>

      {os === 'windows' && (
        <div className={styles.warning}>
          <div className={styles.warningIcon}>⚠️</div>
          <div className={styles.warningContent}>
            <div className={styles.warningTitle}>Windows Limitation</div>
            <div className={styles.warningText}>
              Aria-Gen2 support is not yet available for Windows users. The
              commands below provide fallback options with Gen1-only features.
            </div>
          </div>
        </div>
      )}

      <div className={styles.commandWrap}>
        <div className={styles.commandHeader}>
          <span className={styles.label}>Install Command</span>
          <button className={styles.copyBtn} onClick={doCopy}>
            {copied ? 'Copied ✓' : 'Copy'}
          </button>
        </div>
        <div className={styles.codeBox}>
          <CodeBlock language={codeLang}>{command}</CodeBlock>
        </div>
      </div>
    </div>
  );
}
