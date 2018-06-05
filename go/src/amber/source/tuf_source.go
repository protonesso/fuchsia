// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package source

import (
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io/ioutil"
	"net/url"
	"os"
	"path"
	"time"

	"amber/lg"
	"amber/pkg"

	"fidl/amber"

	tuf "github.com/flynn/go-tuf/client"
	tuf_data "github.com/flynn/go-tuf/data"

	"github.com/flynn/go-tuf/verify"
)

const (
	configFileName = "config.json"
)

// ErrTufSrcNoHash is returned if the TUF entry doesn't have a SHA512 hash
var ErrTufSrcNoHash = errors.New("tufsource: hash missing or wrong type")

// TUFSource wraps a TUF Client into the Source interface
type TUFSource struct {
	client     *tuf.Client
	ratePeriod time.Duration
	rateLimit  uint64
	Store      string
	keys       []*tuf_data.Key
	Config     *amber.SourceConfig
}

type merkle struct {
	Root string `json:"merkle"`
}

type RemoteStoreError struct {
	error
}

type IOError struct {
	error
}

func LoadTUFSourceConfig(path string) (*amber.SourceConfig, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	var cfg amber.SourceConfig
	if err := json.NewDecoder(f).Decode(&cfg); err != nil {
		return nil, err
	}

	return &cfg, nil
}

func NewTUFSource(store string, cfg *amber.SourceConfig) (*TUFSource, error) {
	if store == "" {
		return nil, fmt.Errorf("tuf store path cannot be empty")
	}

	if cfg.Id == "" {
		return nil, fmt.Errorf("tuf source id cannot be empty")
	}

	if _, err := url.ParseRequestURI(cfg.RepoUrl); err != nil {
		return nil, err
	}

	if len(cfg.RootKeys) == 0 {
		return nil, fmt.Errorf("no root keys provided")
	}

	keys := make([]*tuf_data.Key, len(cfg.RootKeys))

	for i, key := range cfg.RootKeys {
		if key.Type != "ed25519" {
			return nil, fmt.Errorf("unsupported key type %s", key.Type)
		}

		keyHex, err := hex.DecodeString(key.Value)
		if err != nil {
			return nil, fmt.Errorf("invalid key value: %s", err)
		}

		keys[i] = &tuf_data.Key{
			Type:  key.Type,
			Value: tuf_data.KeyValue{tuf_data.HexBytes(keyHex)},
		}
	}

	src := TUFSource{
		ratePeriod: time.Millisecond * time.Duration(cfg.RatePeriod),
		rateLimit:  cfg.RateLimit,
		Store:      store,
		keys:       keys,
		Config:     cfg,
	}

	return &src, nil
}

func (f *TUFSource) initSrc() error {
	if f.client != nil {
		return nil
	}

	// We might have multiple things in the store directory, so put tuf in
	// it's own directory.
	tufStore := path.Join(f.Store, "tuf")

	client, store, err := newTUFClient(f.Config.RepoUrl, tufStore)

	if err != nil {
		return err
	}

	needs, err := needsInit(store)
	if err != nil {
		return fmt.Errorf("source status check failed: %s", err)
	}

	if needs {
		err := client.Init(f.keys, len(f.keys))
		if err != nil {
			return fmt.Errorf("TUF init failed: %s", err)
		}
	}

	f.client = client
	return nil
}

func (f *TUFSource) Id() string {
	return f.Config.Id
}

func (f *TUFSource) GetConfig() *amber.SourceConfig {
	return f.Config
}

// AvailableUpdates takes a list of Packages and returns a map from those Packages
// to any available update Package
func (f *TUFSource) AvailableUpdates(pkgs []*pkg.Package) (map[pkg.Package]pkg.Package, error) {
	if err := f.initSrc(); err != nil {
		return nil, fmt.Errorf("tuf_source: source could not be initialized: %s", err)
	}
	_, err := f.client.Update()
	if err != nil && !tuf.IsLatestSnapshot(err) {
		if _, ok := err.(tuf.ErrDecodeFailed); ok {
			e := err.(tuf.ErrDecodeFailed)
			if _, ok := e.Err.(verify.ErrLowVersion); ok {
				err = fmt.Errorf("tuf_source: verify update repository is current or reset "+
					"device storage %s", err)
			}
		}
		return nil, err
	}

	// TODO(jmatt) seems like 'm' should be the same as returned from
	// Client.Update, but empirically this seems untrue, investigate
	m, err := f.client.Targets()

	if err != nil {
		return nil, err
	}

	updates := make(map[pkg.Package]pkg.Package)

	for _, p := range pkgs {
		meta, ok := m[p.Name]
		if !ok {
			continue
		}
		hash, ok := meta.Hashes["sha512"]
		if !ok {
			continue
		}
		hashStr := hash.String()

		m := merkle{}
		if meta.Custom != nil {
			json.Unmarshal(*meta.Custom, &m)
		}

		if (len(p.Version) == 0 || p.Version == hashStr) &&
			(len(p.Merkle) == 0 || p.Merkle == m.Root) {
			lg.Log.Printf("tuf_source: available update %s version %s\n",
				p.Name, hashStr[:8])
			updates[*p] = pkg.Package{
				Name:    p.Name,
				Version: hashStr,
				Merkle:  m.Root,
			}
		}
	}

	return updates, nil
}

// create a wrapper for File so it conforms to interface Client.Download expects
type delFile struct {
	*os.File
}

// Delete removes the file from the filesystem
func (f *delFile) Delete() error {
	f.Close()
	return os.Remove(f.Name())
}

// FetchPkg gets the content for the requested Package
func (f *TUFSource) FetchPkg(pkg *pkg.Package) (*os.File, error) {
	if err := f.initSrc(); err != nil {
		return nil, fmt.Errorf("tuf_source: source could not be initialized: %s", err)
	}
	lg.Log.Printf("tuf_source: requesting download for: %s\n", pkg.Name)
	tmp, err := ioutil.TempFile("", pkg.Version)
	if err != nil {
		return nil, err
	}

	err = f.client.Download(pkg.Name, &delFile{tmp})
	if err != nil {
		return nil, ErrNoUpdateContent
	}

	_, err = tmp.Seek(0, os.SEEK_SET)
	if err != nil {
		tmp.Close()
		os.Remove(tmp.Name())
		return nil, err
	}
	return tmp, nil
}

// CheckInterval returns the time between which checks should be spaced.
func (f *TUFSource) CheckInterval() time.Duration {
	// TODO(jmatt) figure out how to establish a real value from the
	// Client we wrap
	return f.ratePeriod
}

func (f *TUFSource) CheckLimit() uint64 {
	return f.rateLimit
}

// Equals returns true if the Source passed in is a pointer to this instance
func (f *TUFSource) Equals(o Source) bool {
	switch o.(type) {
	case *TUFSource:
		return f == o.(*TUFSource)
	default:
		return false
	}
}

func (s *TUFSource) Save() error {
	// Ignore errors if the data dir already exists
	err := os.MkdirAll(s.Store, os.ModePerm)
	if err != nil {
		return err
	}

	p := path.Join(s.Store, configFileName)

	// We want to atomically write the config, so we'll first write it to a
	// temp file, then do an atomic rename to overwrite the target.

	f, err := ioutil.TempFile(s.Store, configFileName)
	if err != nil {
		return err
	}
	defer f.Close()

	// Make sure to clean up the temp file if there's an error.
	defer func() {
		if err != nil {
			os.Remove(f.Name())
		}
	}()

	// Encode the cfg as a pretty printed json.
	encoder := json.NewEncoder(f)
	encoder.SetIndent("", "    ")

	if err = encoder.Encode(s.Config); err != nil {
		return err
	}

	if err = f.Close(); err != nil {
		return err
	}

	if err = os.Rename(f.Name(), p); err != nil {
		return err
	}

	return nil
}

func newTUFClient(url string, path string) (*tuf.Client, tuf.LocalStore, error) {
	tufStore, err := tuf.FileLocalStore(path)
	if err != nil {
		return nil, nil, IOError{fmt.Errorf("couldn't open datastore: %s", err)}
	}

	server, err := tuf.HTTPRemoteStore(url, nil, nil)
	if err != nil {
		return nil, nil, RemoteStoreError{fmt.Errorf("server address not understood: %s", err)}
	}

	return tuf.NewClient(tufStore, server), tufStore, nil
}

func needsInit(s tuf.LocalStore) (bool, error) {
	meta, err := s.GetMeta()
	if err != nil {
		return false, err
	}

	_, ok := meta["root.json"]
	return !ok, nil
}
