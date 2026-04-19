import React from 'react';
import {useThemeConfig, ErrorCauseBoundary} from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import { useAuth } from '../../../contexts/AuthContext';
import styles from './styles.module.css';

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items;
}

function NavbarItems({items}) {
  return (
    <>
      {items.map((item, i) => (
        <ErrorCauseBoundary
          key={i}
          onError={(error) =>
            new Error(
              `A theme navbar item failed to render.
Please double-check the following navbar item (themeConfig.navbar.items) of your Docusaurus config:
${JSON.stringify(item, null, 2)}`,
              {cause: error},
            )
          }>
          <NavbarItem {...item} />
        </ErrorCauseBoundary>
      ))}
    </>
  );
}

function UserNavbarItems() {
  // Handle cases where AuthContext might not be available (SSR)
  let authState;
  try {
    authState = useAuth();
  } catch (error) {
    // If AuthContext is not available, show default signin/signup buttons
    console.warn('AuthContext not available in navbar, showing default auth buttons');
    return (
      <>
        <NavbarItem
          href="/auth/signin"
          label="Sign In"
          position="right"
        />
        <NavbarItem
          href="/auth/signup"
          label="Sign Up"
          position="right"
        />
      </>
    );
  }

  const { isAuthenticated, user, logout, loading } = authState;

  // Show loading state
  if (loading) {
    return (
      <div className="navbar__item">
        <span className={`navbar__link ${styles.loadingText}`}>
          Loading...
        </span>
      </div>
    );
  }

  // Show user info when authenticated
  if (isAuthenticated && user) {
    return (
      <>
        <div className={`navbar__item dropdown dropdown--hoverable ${styles.userDropdown}`}>
          <a
            href="#"
            className="navbar__link"
            aria-haspopup="true"
            aria-expanded="false"
            role="button"
            onClick={(e) => e.preventDefault()}
          >
            <span className={styles.userIcon}>👤</span>
            {user.full_name || user.name || user.email}
          </a>
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="/auth/profile">
                Profile
              </a>
            </li>
            <li>
              <button
                className="dropdown__link"
                onClick={logout}
                style={{
                  background: 'none',
                  border: 'none',
                  width: '100%',
                  textAlign: 'left',
                  cursor: 'pointer'
                }}
              >
                Sign Out
              </button>
            </li>
          </ul>
        </div>
      </>
    );
  }

  // Show sign in/up buttons when not authenticated
  return (
    <>
      <NavbarItem
        href="/auth/signin"
        label="Sign In"
        position="right"
      />
      <NavbarItem
        href="/auth/signup"
        label="Sign Up"
        position="right"
      />
    </>
  );
}

function NavbarContentLayout({left, right}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

export default function NavbarContent() {
  const mobileSidebar = useNavbarMobileSidebar();
  const items = useNavbarItems();

  // Filter out the static sign in/up items since we'll handle them dynamically
  const filteredItems = items.filter(item =>
    item.href !== '/auth/signin' &&
    item.href !== '/auth/signup' &&
    item.label !== 'Sign In' &&
    item.label !== 'Sign Up'
  );

  const [leftItems, rightItems] = splitNavbarItems(filteredItems);
  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        // TODO stop hardcoding items?
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        // TODO stop hardcoding items?
        // Ask the user to add the respective navbar items => more flexible
        <>
          <NavbarItems items={rightItems} />
          <UserNavbarItems />
          <NavbarColorModeToggle className={styles.colorModeToggle} />
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}
        </>
      }
    />
  );
}
